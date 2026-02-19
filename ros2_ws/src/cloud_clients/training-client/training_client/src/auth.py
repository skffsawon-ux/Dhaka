"""
OIDC robot authentication client.

Handles service-key → JWT exchange via OpenID Connect discovery,
and provides transparent token renewal.

Usage::

    from training_client.src.auth import AuthProvider

    auth = AuthProvider(issuer_url="https://auth-v1.innate.bot", service_key="sk_...")
    token = auth.token                  # lazily discovers + fetches on first access
    auth.token_needs_renewal = True     # mark expired; next .token call will refresh
    token = auth.token                  # transparently renewed

Based on innate-auth robot client (innate_auth_robot.py).
"""
from __future__ import annotations

import logging
import threading

import requests

logger = logging.getLogger(__name__)


class AuthError(Exception):
    """Raised when authentication or token renewal fails."""

    def __init__(self, message: str, status_code: int | None = None) -> None:
        self.status_code = status_code
        super().__init__(message)


class AuthProvider:
    """Exchanges an innate service key for a JWT and handles renewal.

    OIDC discovery is deferred until the first token request.  Setting
    :attr:`token_needs_renewal` to ``True`` causes the next :attr:`token`
    access to fetch a fresh JWT.

    Thread-safe: concurrent access is serialised by an internal lock.
    """

    def __init__(self, issuer_url: str, service_key: str) -> None:
        self._issuer_url = issuer_url.rstrip("/")
        self._service_key = service_key
        self._token: str | None = None
        self._token_endpoint: str | None = None
        self._lock = threading.Lock()
        self.token_needs_renewal: bool = True  # force fetch on first access

    # ── Public API ──────────────────────────────────────────────────

    @property
    def token(self) -> str:
        """Return the current JWT, discovering + fetching as needed."""
        if self.token_needs_renewal or self._token is None:
            return self._renew()
        return self._token

    # ── Internals ───────────────────────────────────────────────────

    def _renew(self) -> str:
        """Discover (if needed) and fetch a fresh JWT."""
        with self._lock:
            # Double-check after acquiring the lock — another thread may
            # have already renewed while we were waiting.
            if not self.token_needs_renewal and self._token is not None:
                return self._token

            if self._token_endpoint is None:
                self._token_endpoint = self._discover_token_endpoint()

            logger.debug("Renewing JWT from %s", self._token_endpoint)
            try:
                resp = requests.post(
                    self._token_endpoint,
                    headers={"Authorization": f"Bearer {self._service_key}"},
                    timeout=10,
                )
                resp.raise_for_status()
            except requests.HTTPError as exc:
                code = exc.response.status_code if exc.response is not None else None
                raise AuthError(
                    f"Token exchange failed: {exc}", status_code=code
                ) from exc
            except requests.RequestException as exc:
                raise AuthError(f"Token exchange request failed: {exc}") from exc

            data = resp.json()
            jwt = data.get("token")
            if not jwt:
                raise AuthError(
                    "Auth response missing 'token' field; "
                    f"got keys: {list(data.keys())}"
                )

            self._token = jwt
            self.token_needs_renewal = False
            logger.info("JWT renewed successfully")
            return jwt

    def _discover_token_endpoint(self) -> str:
        """Fetch the OIDC discovery document and extract the token endpoint."""
        url = f"{self._issuer_url}/.well-known/openid-configuration"
        logger.debug("Discovering OIDC config from %s", url)
        try:
            resp = requests.get(url, timeout=10)
            resp.raise_for_status()
        except requests.RequestException as exc:
            raise AuthError(
                f"OIDC discovery failed at {url}: {exc}"
            ) from exc

        discovery = resp.json()
        endpoint = discovery.get("token_endpoint")
        if not endpoint:
            raise AuthError(
                "OIDC discovery response missing 'token_endpoint'; "
                f"got keys: {list(discovery.keys())}"
            )

        logger.debug("Discovered token endpoint: %s", endpoint)
        return endpoint

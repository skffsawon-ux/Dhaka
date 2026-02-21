"""Cartesia TTS adapter for the Innate proxy client.

The adapter expects a *parent* object that exposes:

- ``parent.request(service_name, endpoint, method, json=...)`` → ``httpx.Response``

This is satisfied by both :class:`innate_proxy.ProxyClient` and
``brain_client.client.proxy_client.ProxyClient``.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Iterator

logger = logging.getLogger(__name__)


class ProxyCartesiaClient:
    """Drop-in Cartesia TTS client that routes through the Innate service proxy."""

    def __init__(self, parent: Any) -> None:
        self._parent = parent

    class TTS:
        """Text-to-speech sub-API."""

        def __init__(self, parent: Any) -> None:
            self._parent = parent

        def bytes(
            self,
            model_id: str,
            transcript: str,
            voice: Dict[str, Any],
            output_format: Dict[str, Any],
        ) -> bytes | Iterator[bytes]:
            """Generate speech audio bytes via the proxy."""
            body = {
                "model_id": model_id,
                "transcript": transcript,
                "voice": voice,
                "output_format": output_format,
            }
            response = self._parent.request(
                service_name="cartesia",
                endpoint="/tts/bytes",
                method="POST",
                json=body,
            )
            return response.content

    @property
    def tts(self) -> TTS:
        """Get the TTS sub-API."""
        return self.TTS(self._parent)

    def close(self) -> None:
        self._parent.close()

    def __enter__(self) -> "ProxyCartesiaClient":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

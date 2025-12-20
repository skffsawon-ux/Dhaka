# Health Monitor API

The robot provides a health monitoring service that reports the status of critical services and captures crash errors. This helps the app show meaningful error messages when services are unavailable.

## Service: `/system/health`

**Type:** `std_srvs/srv/Trigger`

Call this service when you detect that a critical service (like `/brain/get_available_directives`) doesn't exist. It returns detailed information about what went wrong.

### Response Format

```json
{
  "healthy": false,
  "timestamp": "2025-12-19T22:30:00.123456",
  "services": {
    "/brain/get_available_directives": false,
    "/brain/get_chat_history": false,
    "/brain/reset_brain": false,
    "/brain/set_brain_active": false
  },
  "nodes": {
    "brain_client_node": false,
    "primitive_execution_action_server": true,
    "rosbridge_websocket": true
  },
  "errors": [
    {
      "source": "brain_client",
      "message": "AttributeError: 'NoneType' object has no attribute 'config'"
    }
  ],
  "summary": "Missing services: /brain/get_available_directives; Crash detected: AttributeError: 'NoneType' object has no attribute 'config'"
}
```

### Fields

| Field | Type | Description |
|-------|------|-------------|
| `healthy` | boolean | `true` if all critical services and nodes are running |
| `timestamp` | string | ISO timestamp of when the check was performed |
| `services` | object | Map of service names to availability (true/false) |
| `nodes` | object | Map of node names to running status (true/false) |
| `errors` | array | Recent crash errors found in logs |
| `summary` | string | Human-readable summary of issues (use this for display) |

## Topic: `/system/health`

**Type:** `std_msgs/msg/String`

The health monitor also publishes to this topic every 5 seconds with the same JSON format. Subscribe to get real-time health updates.

## Usage Example (TypeScript/React Native)

```typescript
import { callService, subscribe } from './rosbridge';

interface HealthStatus {
  healthy: boolean;
  timestamp: string;
  services: Record<string, boolean>;
  nodes: Record<string, boolean>;
  errors: Array<{ source: string; message: string }>;
  summary: string;
}

// Call when a service doesn't exist
async function getHealthStatus(): Promise<HealthStatus> {
  const response = await callService(
    '/system/health',
    'std_srvs/srv/Trigger',
    {}
  );
  return JSON.parse(response.message);
}

// Example: Enhanced error handling for brain services
async function getDirectives() {
  try {
    return await callService(
      '/brain/get_available_directives',
      'brain_messages/srv/GetAvailableDirectives',
      {}
    );
  } catch (error) {
    if (error.message?.includes('does not exist')) {
      // Service doesn't exist - get detailed health info
      try {
        const health = await getHealthStatus();
        if (!health.healthy) {
          // Show the summary which includes crash details
          throw new Error(`System Error: ${health.summary}`);
        }
      } catch (healthError) {
        // Health service also unavailable - system is very broken
        throw new Error('Robot system is not responding. Please restart.');
      }
    }
    throw error;
  }
}
```

## Common Error Scenarios

### 1. Brain Node Crashed on Startup
```json
{
  "healthy": false,
  "summary": "Missing nodes: brain_client_node; Crash detected: AttributeError: 'NoneType' object has no attribute 'config'"
}
```
**User-friendly message:** "The brain service crashed during startup. Check robot logs for details."

### 2. Services Not Yet Available (Still Starting)
```json
{
  "healthy": false,
  "summary": "Missing services: /brain/get_available_directives, /brain/reset_brain"
}
```
**User-friendly message:** "Robot is still starting up. Please wait a moment and try again."

### 3. All Systems Healthy
```json
{
  "healthy": true,
  "summary": "All systems operational"
}
```

## Monitored Services

- `/brain/get_available_directives`
- `/brain/get_chat_history`
- `/brain/reset_brain`
- `/brain/set_brain_active`

## Monitored Nodes

- `brain_client_node`
- `primitive_execution_action_server`
- `rosbridge_websocket`


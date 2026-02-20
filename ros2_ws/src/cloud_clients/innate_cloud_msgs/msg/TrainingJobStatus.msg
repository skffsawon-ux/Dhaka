# Status snapshot for a single training run.

# Run status constants (match server-side RunStatus enum).
uint8 STATUS_UNKNOWN=0
uint8 STATUS_WAITING_FOR_APPROVAL=1
uint8 STATUS_APPROVED=2
uint8 STATUS_REJECTED=3
uint8 STATUS_BOOTING=4
uint8 STATUS_RUNNING=5
uint8 STATUS_DONE=6
uint8 STATUS_DOWNLOADED=7

int32 run_id
uint8 status
string daemon_state         # remote daemon sub-state (too dynamic to enum)
string error_message

builtin_interfaces/Time started_at   # zero if unset
builtin_interfaces/Time finished_at  # zero if unset
string instance_type

# Download progress for this run (e.g. downloading results).
bool has_active_transfer
innate_cloud_msgs/TransferProgress active_transfer

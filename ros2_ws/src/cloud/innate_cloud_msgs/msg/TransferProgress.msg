# Per-file upload / download byte-level progress.

# Direction
uint8 UPLOAD=0
uint8 DOWNLOAD=1
uint8 direction

# Stage
uint8 STAGE_COMPRESSING=0
uint8 STAGE_UPLOADING=1
uint8 STAGE_DOWNLOADING=2
uint8 STAGE_VERIFYING=3
uint8 STAGE_DONE=4
uint8 STAGE_ERROR=5
uint8 stage

string filename
int32 file_index            # 1-based position in the batch
int32 file_total            # total files in the batch
int64 bytes_done
int64 bytes_total
bool file_done              # this individual file is finished
bool transfer_done          # entire transfer operation is finished
string error
string message              # human-readable summary

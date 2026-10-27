# Structured mission status for the Mission Commander.
# Numeric fields are taken directly from the Nav2 NavigateToPose action feedback
# (nav2_msgs/action/NavigateToPose.Feedback).

uint8 IDLE = 0
uint8 NAVIGATING = 1
uint8 SUCCEEDED = 2
uint8 FAILED = 3
uint8 CANCELED = 4

uint8 status                        # one of IDLE/NAVIGATING/SUCCEEDED/FAILED/CANCELED
string target_name                  # symbolic name of the target location (e.g., "room_a")
float32 distance_remaining          # meters, from NavigateToPose.Feedback.distance_remaining
float32 estimated_time_remaining    # seconds, from NavigateToPose.Feedback.estimated_time_remaining
int16 number_of_recoveries          # from NavigateToPose.Feedback.number_of_recoveries
float32 elapsed_time                # seconds, from NavigateToPose.Feedback.navigation_time
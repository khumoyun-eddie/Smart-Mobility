from lane_keeping.main import PIDController

def test_pid_controller():
    # Initialize PID controller with test values
    pid = PIDController(kP=0.1, kI=0.01, kD=0.05, kS=5)

    # Simulate control loop
    error = 10.0  # Some error value
    control_output = pid.control(error, timestamp=1.0)  # Provide a timestamp for control

    # Check if control output is within expected range or behavior
    assert isinstance(control_output, float)  # Ensure control output is a float or the expected type
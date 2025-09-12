from y1_sdk import Y1SDKInterface, ControlMode

sdk = Y1SDKInterface(can_id="can0", urdf_path="path/to/urdf", arm_end_type=0, enable_arm=True)
sdk.Init()
extends Node3D

@onready var right_controller: XRController3D = $XROrigin3D/RightController


func _ready() -> void:
	var xr_interface := XRServer.find_interface("OpenXR")
	if xr_interface == null:
		push_error("OpenXR interface was not found.")
		return

	if not xr_interface.is_initialized():
		if not xr_interface.initialize():
			push_error("OpenXR initialization failed.")
			return

	get_viewport().use_xr = true
	print("OpenXR initialized. Sending right controller pose to UDP.")


func _exit_tree() -> void:
	get_viewport().use_xr = false

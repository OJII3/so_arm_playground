extends XRController3D

@export var udp_host := "127.0.0.1"
@export var udp_port := 50530
@export var source_name := "quest3/right_controller"
@export var send_hz := 60.0

var _udp := PacketPeerUDP.new()
var _seq := 0
var _accumulator := 0.0


func _ready() -> void:
	var err := _udp.connect_to_host(udp_host, udp_port)
	if err != OK:
		push_error("Failed to connect UDP pose sender: %s" % err)


func _physics_process(delta: float) -> void:
	_accumulator += delta
	var interval := 1.0 / send_hz
	if _accumulator < interval:
		return
	_accumulator = 0.0
	_send_pose()


func _send_pose() -> void:
	_seq += 1
	var basis := global_transform.basis
	var quat := basis.get_rotation_quaternion()
	var enabled := is_button_pressed("grip_click") or get_float("grip") > 0.35
	var payload := {
		"version": 1,
		"seq": _seq,
		"timestamp_usec": Time.get_ticks_usec(),
		"source": source_name,
		"position": [
			global_position.x,
			global_position.y,
			global_position.z
		],
		"orientation_xyzw": [
			quat.x,
			quat.y,
			quat.z,
			quat.w
		],
		"trigger": clampf(get_float("trigger"), 0.0, 1.0),
		"grip": clampf(get_float("grip"), 0.0, 1.0),
		"enabled": enabled
	}
	var bytes := JSON.stringify(payload).to_utf8_buffer()
	_udp.put_packet(bytes)

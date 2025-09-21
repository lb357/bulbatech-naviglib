extends SubViewport

@export var target: Node3D
@export var target_position_offset: Vector3
@export var target_rotation_offset: Vector3
@onready var camera: Camera3D = $Camera


func _process(delta: float) -> void:
	camera.global_position = target.global_position + target_position_offset
	camera.global_rotation_degrees = target.global_rotation_degrees + target_rotation_offset

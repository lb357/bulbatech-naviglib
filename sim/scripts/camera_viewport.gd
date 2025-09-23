extends SubViewport

@export var target: Node3D
@onready var camera: Camera3D = $Camera


func _process(delta: float) -> void:
	camera.global_transform = target.global_transform

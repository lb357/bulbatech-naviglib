extends CharacterBody3D


@onready var Camera = $Camera

@export_range(1, 100) var Camera_Sensevity: int = 8# / 1000
@export_range(1, 20) var Camera_Speed: int = 10

var is_operating = false

func _input(e: InputEvent) -> void:
	if e is InputEventMouseButton:
		if e.button_index == MOUSE_BUTTON_RIGHT and e.pressed:
			changeState()
			
		elif e.button_index == MOUSE_BUTTON_RIGHT and e.is_released():
			changeState()
			
		if e.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			Camera_Speed -= 1 if Camera_Speed - 1 > 0 else 0
			
		elif e.button_index == MOUSE_BUTTON_WHEEL_UP:
			Camera_Speed += 1 if Camera_Speed + 1 <= 20 else 0
			
	if !is_operating: return
			
	if e is InputEventMouseMotion:
		var rot_x = -e.relative.y * Camera_Sensevity /1000
		var rot_y = -e.relative.x * Camera_Sensevity /1000
		
		Camera.rotate_x(rot_x)
		
		Camera.rotation.x = clamp(Camera.rotation.x, -1.25, 1.25)
		
		rotate_y(rot_y)
		
func changeState() -> void:
	is_operating = !is_operating
	
	Input.mouse_mode = Input.MOUSE_MODE_CAPTURED if is_operating else Input.MOUSE_MODE_VISIBLE
	
	
func _physics_process(delta: float) -> void:
	
	if !is_operating:
		velocity = Vector3.ZERO
		return
	
	var input_dir_horizontal = Input.get_vector("mv_lt", "mv_rt","mv_fd","mv_bw")
	var input_dir_vertical = Input.get_axis("mv_dw", "mv_up")

	var direction = (transform.basis * Vector3(input_dir_horizontal.x, input_dir_vertical, input_dir_horizontal.y)).normalized()
	
	if direction:
		velocity.x = direction.x * Camera_Speed
		velocity.z = direction.z * Camera_Speed
		velocity.y = direction.y * Camera_Speed
	else:
		velocity.x = move_toward(velocity.x, 0, Camera_Speed)
		velocity.z = move_toward(velocity.z, 0, Camera_Speed)
		velocity.y = move_toward(velocity.y, 0, Camera_Speed)
	
	move_and_slide()
	
	

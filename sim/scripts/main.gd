extends Node3D

@export var vp_list: Array[Viewport]
@export var img_format: Image.Format = Image.FORMAT_L8

const PORT = 9080
var tcp_server = TCPServer.new()
var peers: Dictionary[int, WebSocketPeer] = {}
var last_peer_id := 1


func _ready():
	var err = tcp_server.listen(PORT)
	
	if err == OK:
		print("Server started.")
	else:
		push_error("Unable to start server.")
		set_process(false)


func _process(_delta):
	while tcp_server.is_connection_available():
		last_peer_id += 1
		print("+ Peer %d connected." % last_peer_id)
		var ws = WebSocketPeer.new()
		ws.accept_stream(tcp_server.take_connection())
		ws.set_outbound_buffer_size(2**64)
		peers[last_peer_id] = ws

	for peer_id in peers.keys():
		var peer = peers[peer_id]

		peer.poll()

		var peer_state = peer.get_ready_state()
		
		if peer_state == WebSocketPeer.STATE_OPEN:
			while peer.get_available_packet_count():
				var packet = peer.get_packet()
				#print("< Got binary data from peer %d with length %d" % [peer_id, packet.size()])
				if packet[0] == 0:
					var vp_id = packet.slice(1, 9).decode_s64(0)
					
					var vp_x = PackedByteArray()
					vp_x.resize(8)
					vp_x.encode_s64(0, vp_list[vp_id].get_size().x)
					
					var vp_y = PackedByteArray()
					vp_y.resize(8)
					vp_y.encode_s64(0, vp_list[vp_id].get_size().y)
					
					var vp_format = PackedByteArray()
					vp_format.resize(8)
					vp_format.encode_s64(0, img_format)
					
					var ping_packet = packet.slice(0, 9)
					ping_packet.append_array(vp_y)
					ping_packet.append_array(vp_x)
					ping_packet.append_array(vp_format)
					peer.send(ping_packet)
				elif packet[0] == 4:
					var vp_id = packet.slice(1, 9).decode_s64(0)
					
					var image_data = get_viewport_data(vp_list[vp_id])
					var image_packet = packet.slice(0, 9)
					image_packet.append_array(image_data.compress(FileAccess.CompressionMode.COMPRESSION_GZIP))
					peer.send(image_packet)
					#print("> Sent binary data to peer %d with length %d" % [peer_id, image_packet.size()])
				
				elif packet[0] == 5:
					# Вне протокола. Показательно для чисел с плавающей точкой
					var double_value = packet.slice(1, 9).decode_double(0)
					
					print(double_value)
			
					var double_pba = PackedByteArray()
					double_pba.resize(8)
					double_pba.encode_double(0, double_value)
					
					var double_packet = packet.slice(0, 1)
					double_packet.append_array(double_pba)
					peer.send(double_packet)
		
		elif peer_state == WebSocketPeer.STATE_CLOSED:
			peers.erase(peer_id)
			var code = peer.get_close_code()
			var reason = peer.get_close_reason()
			print("- Peer %s closed with code: %d, reason %s. Clean: %s" % [peer_id, code, reason, code != -1])
			
			
func get_viewport_data(viewport: Viewport, image_format: int = img_format) -> PackedByteArray:
	var texture = viewport.get_texture()
	var image = texture.get_image()
	image.convert(image_format)
	return image.get_data()

uint64 timestamp	# time since system start (microseconds)
uint16 payload_type  # A code that identifies the content of the payload (0 for unknown, which is the default). If this code is less than 32768, it is a 'registered' payload type and the corresponding code should be added to the MAV_TUNNEL_PAYLOAD_TYPE enum. Software creators can register blocks of types as needed. Codes greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.*/
uint8 target_system  # System ID (can be 0 for broadcast, but this is discouraged)
uint8 target_component  # Component ID (can be 0 for broadcast, but this is discouraged)*/
uint8 payload_length  #  Length of the data transported in payload*/
uint8[128] payload #

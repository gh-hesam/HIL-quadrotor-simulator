import redis

redis_server = redis.Redis(
            host='127.0.0.1',
            port=6379, 
            password='')

def get_command_from_controller(redis_key):
    commands = redis_server.get(redis_key)
    commands = commands.decode('utf-8')
    
def set_commands(redis_key, roll, pitch, yaw, T):
    cmd = str(roll) + ',' +str(pitch) + ',' + str(yaw) + ','+ str(T)
    redis_server.set(redis_key, cmd)

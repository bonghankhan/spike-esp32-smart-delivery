# SPIKE Prime autonomous delivery rover
# Target: LEGO Education SPIKE App 3 Python runtime
# Ports: left motor A, right motor B, distance sensor C

from hub import port, light_matrix
import runloop
import motor_pair
import distance_sensor

PAIR = motor_pair.PAIR_1
LEFT_MOTOR = port.A
RIGHT_MOTOR = port.B
DISTANCE = port.C

STOP_DISTANCE_MM = 180
DRIVE_VELOCITY = 350
TURN_VELOCITY = 260

async def main():
    motor_pair.pair(PAIR, LEFT_MOTOR, RIGHT_MOTOR)
    light_matrix.show_image(light_matrix.IMAGE_HAPPY)

    while True:
        distance = distance_sensor.distance(DISTANCE)

        # -1 means no valid target was detected.
        if distance == -1 or distance > STOP_DISTANCE_MM:
            motor_pair.move(PAIR, 0, velocity=DRIVE_VELOCITY)
            await runloop.sleep_ms(50)
            continue

        # Obstacle detected: stop, reverse a little, then arc-turn.
        motor_pair.stop(PAIR)
        light_matrix.show_image(light_matrix.IMAGE_NO)
        await runloop.sleep_ms(200)

        await motor_pair.move_for_degrees(PAIR, -180, 0, velocity=220)
        await motor_pair.move_for_degrees(PAIR, 330, 100, velocity=TURN_VELOCITY)

        light_matrix.show_image(light_matrix.IMAGE_HAPPY)
        await runloop.sleep_ms(100)

runloop.run(main())

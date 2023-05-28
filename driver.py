from pynput import keyboard
import flywheel as f
# import physics as p

keys = [keyboard.KeyCode.from_char('w') , keyboard.KeyCode.from_char('a'),keyboard.KeyCode.from_char('s'),keyboard.KeyCode.from_char('d'),keyboard.KeyCode.from_char('l'),keyboard.KeyCode.from_char('j'),keyboard.Key.shift]
keysPressing = [False,False,False,False,False,False]
accel = 0.8
ts = 128.7
sx = 100
sy = 10

def on_press(key):
    global accel,keys
    # if (key in keys):
    #     if key == keys[0]:
    #         p.leftVelocity += accel
    #         p.rightVelocity += accel
    #     if key == keys[1]:
    #         p.leftVelocity += accel
    #         p.rightVelocity -=accel
    #     if key == keys[2]:
    #         p.leftVelocity -= accel
    #         p.rightVelocity -= accel
    #     if key == keys[3]:
    #         p.leftVelocity -= accel
    #         p.rightVelocity += accel

    if (key in keys):
        if key == keys[0]:
            keysPressing[0] = True
        if key == keys[1]:
            keysPressing[1] = True
        if key == keys[2]:
            keysPressing[2] = True
        if key == keys[3]:
            keysPressing[3] = True
        if key == keys[4]:
            keysPressing[4] = True
        if key == keys[5]:
            keysPressing[5] = True
        if key == keys[6]:
            f.shoot(1,(sx + 3/4*ts, sy + 5*ts + 1/4*ts), 10, 0.14)



def on_release(key):
    if (key in keys):
        if key == keys[0]:
            keysPressing[0] = False
        if key == keys[1]:
            keysPressing[1] = False
        if key == keys[2]:
            keysPressing[2] = False
        if key == keys[3]:
            keysPressing[3] = False
        if key == keys[4]:
            keysPressing[4] = False
        if key == keys[5]:
            keysPressing[5] = False

    if key == keyboard.Key.esc:
        return False

listener = keyboard.Listener(on_press=on_press,on_release=on_release)
listener.start()
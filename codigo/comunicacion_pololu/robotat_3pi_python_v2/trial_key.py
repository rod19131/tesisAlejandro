import keyboard
cont = 0
while True:
    cont = cont + 1
    print(cont)
    if keyboard.is_pressed('a'):
        break
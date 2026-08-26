#!/usr/bin/env python3
"""Send held normalized mecanum commands to the ESP32 UDP controller."""
import argparse, select, socket, sys, termios, time, tty

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.19.137")
    parser.add_argument("--port", type=int, default=3333)
    parser.add_argument("--speed", type=float, default=.12)
    parser.add_argument("--turn-speed", type=float)
    args=parser.parse_args()
    if args.turn_speed is None:
        args.turn_speed=args.speed
    commands={"w":(args.speed,0,0),"s":(-args.speed,0,0),"a":(0,-args.speed,0),"d":(0,args.speed,0),"q":(0,0,-args.turn_speed),"e":(0,0,args.turn_speed)," ":(0,0,0)}
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); old=termios.tcgetattr(sys.stdin)
    print("W/S forward/reverse, A/D strafe, Q/E rotate, Space stop, Esc quit")
    try:
        tty.setcbreak(sys.stdin.fileno()); command=(0,0,0)
        while True:
            ready,_,_=select.select([sys.stdin],[],[],.05)
            if ready:
                key=sys.stdin.read(1).lower()
                if key=="\x1b": sock.sendto(b"stop",(args.host,args.port)); break
                command=commands.get(key,command)
            sock.sendto("drive {:.2f} {:.2f} {:.2f}".format(*command).encode(),(args.host,args.port)); time.sleep(.05)
    finally:
        sock.sendto(b"stop",(args.host,args.port))
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,old); sock.close()
if __name__=="__main__": main()

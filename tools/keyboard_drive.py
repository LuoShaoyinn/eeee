#!/usr/bin/env python3
"""Send keyboard commands to the ESP32 UDP controller."""
import argparse, select, socket, sys, termios, time, tty

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.19.137")
    parser.add_argument("--port", type=int, default=3333)
    parser.add_argument("--speed", type=float, default=.12)
    parser.add_argument("--turn-speed", type=float)
    parser.add_argument("--s3", action="store_true", help="test positional S3 instead of driving the chassis")
    parser.add_argument("--ga25", action="store_true", help="test the GA25-370 on the external L298N")
    parser.add_argument("--s3-step", type=int, default=5)
    parser.add_argument("--imu", action="store_true", help="watch IMU telemetry until Ctrl-C")
    parser.add_argument("--imu-interval", type=float, default=.5, help="IMU query interval in seconds")
    args=parser.parse_args()
    if sum((args.s3, args.ga25, args.imu)) > 1:
        parser.error("--s3, --ga25, and --imu cannot be combined")
    if args.imu_interval <= 0:
        parser.error("--imu-interval must be positive")
    if args.turn_speed is None:
        args.turn_speed=args.speed
    commands={"w":(args.speed,0,0),"s":(-args.speed,0,0),"a":(0,-args.speed,0),"d":(0,args.speed,0),"q":(0,0,-args.turn_speed),"e":(0,0,args.turn_speed)," ":(0,0,0)}
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    def send(text):
        sock.sendto(text.encode(),(args.host,args.port))
        sock.settimeout(.25)
        try:
            print(sock.recvfrom(256)[0].decode(errors="replace").strip())
        except socket.timeout:
            print("no ESP32 reply")
        finally:
            sock.settimeout(None)
    if args.imu:
        print("Waiting for ESP32 IMU telemetry; press Ctrl-C to stop")
        try:
            while True:
                send("imu")
                time.sleep(args.imu_interval)
        except KeyboardInterrupt:
            pass
        finally:
            sock.close()
        return
    old=termios.tcgetattr(sys.stdin)
    if args.s3:
        if args.s3_step < 1 or args.s3_step > 30:
            parser.error("--s3-step must be 1..30")
        print("S3 test: A/D move target, 0 set 0 deg, C center 60 deg, Space release, Esc quit")
    elif args.ga25:
        print("GA25 L298N test: A/D change PWM by 5%, 0 or Space stop, Esc quit")
    else:
        print("W/S forward/reverse, A/D strafe, Q/E rotate, Space stop, Esc quit")
    try:
        tty.setcbreak(sys.stdin.fileno()); command=(0,0,0); s3_angle=0; ga25_duty=0
        while True:
            ready,_,_=select.select([sys.stdin],[],[],.05)
            if ready:
                key=sys.stdin.read(1).lower()
                if key=="\x1b":
                    send("s3 release" if args.s3 else "ga25 0" if args.ga25 else "stop")
                    break
                if args.s3:
                    if key=="a": s3_angle=max(0,s3_angle-args.s3_step); send(f"s3 {s3_angle}")
                    elif key=="d": s3_angle=min(120,s3_angle+args.s3_step); send(f"s3 {s3_angle}")
                    elif key=="0": s3_angle=0; send("s3 0")
                    elif key=="c": s3_angle=60; send("s3 center")
                    elif key==" ": send("s3 release")
                    continue
                if args.ga25:
                    if key=="a": ga25_duty=max(-100,ga25_duty-args.s3_step)
                    elif key=="d": ga25_duty=min(100,ga25_duty+args.s3_step)
                    elif key in ("0", " "): ga25_duty=0
                    else: continue
                    send(f"ga25 {ga25_duty}")
                    continue
                command=commands.get(key,command)
            if args.ga25:
                sock.sendto(f"ga25 {ga25_duty}".encode(),(args.host,args.port)); time.sleep(.1)
            else:
                sock.sendto("drive {:.2f} {:.2f} {:.2f}".format(*command).encode(),(args.host,args.port)); time.sleep(.05)
    finally:
        sock.sendto(b"s3 release" if args.s3 else b"ga25 0" if args.ga25 else b"stop",(args.host,args.port))
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,old); sock.close()
if __name__=="__main__": main()

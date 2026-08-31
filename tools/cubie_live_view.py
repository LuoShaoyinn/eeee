"""View Cubie's latest annotated optical-flow frame locally over SSH."""
import argparse
import getpass
import io
import time

import cv2
import numpy as np
import paramiko


parser = argparse.ArgumentParser()
parser.add_argument("--host", default="192.168.19.105")
parser.add_argument("--user", default="radxa")
parser.add_argument("--remote", default="/home/radxa/optical_flow/live_preview.jpg")
args = parser.parse_args()

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(args.host, username=args.user, password=getpass.getpass("Cubie SSH password: "),
               timeout=10, look_for_keys=False, allow_agent=False)
sftp = client.open_sftp()
try:
    while True:
        buffer = io.BytesIO()
        try:
            sftp.getfo(args.remote, buffer)
            payload = buffer.getvalue()
            if not payload:
                continue
            image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
            if image is not None:
                cv2.imshow("Cubie optical flow (SSH)", image)
        except OSError:
            pass
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break
        time.sleep(0.04)
finally:
    sftp.close()
    client.close()
    cv2.destroyAllWindows()

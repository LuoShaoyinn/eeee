"""Install the Cubie deployment key, inspect its camera, and deploy the vision service."""

import argparse
import getpass
from pathlib import Path

import paramiko


ROOT = Path(__file__).resolve().parent.parent
REMOTE_DIR = "/home/radxa/optical_flow"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.19.105")
    parser.add_argument("--user", default="radxa")
    parser.add_argument("--password", help="SSH/sudo password; prompted when omitted")
    parser.add_argument("--key", type=Path, default=Path.home() / ".ssh" / "codex_cubie_ed25519")
    parser.add_argument("--install-key", action="store_true")
    parser.add_argument("--inspect", action="store_true")
    parser.add_argument("--deploy", action="store_true")
    return parser.parse_args()


def connect(args: argparse.Namespace, password: str) -> paramiko.SSHClient:
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(args.host, username=args.user,
                       pkey=paramiko.Ed25519Key.from_private_key_file(str(args.key)),
                       timeout=12, look_for_keys=False, allow_agent=False)
    except (paramiko.SSHException, OSError):
        client.connect(args.host, username=args.user, password=password,
                       timeout=12, look_for_keys=False, allow_agent=False)
    return client


def run(client: paramiko.SSHClient, command: str, stdin_text: str | None = None) -> str:
    stdin, stdout, stderr = client.exec_command(command)
    if stdin_text is not None:
        stdin.write(stdin_text)
        stdin.flush()
        stdin.channel.shutdown_write()
    code = stdout.channel.recv_exit_status()
    output = stdout.read().decode(errors="replace") + stderr.read().decode(errors="replace")
    if code:
        raise RuntimeError(f"Remote command failed ({code}):\n{output}")
    return output


def install_key(client: paramiko.SSHClient, key: Path) -> None:
    public_key = key.with_suffix(".pub")
    if not public_key.is_file():
        raise FileNotFoundError(f"Public key not found: {public_key}")
    remote_public = "/tmp/codex_cubie_deploy.pub"
    sftp = client.open_sftp()
    try:
        sftp.put(str(public_key), remote_public)
    finally:
        sftp.close()
    run(client, "mkdir -p ~/.ssh && chmod 700 ~/.ssh && touch ~/.ssh/authorized_keys "
                "&& chmod 600 ~/.ssh/authorized_keys && "
                f"(grep -qxF -f {remote_public} ~/.ssh/authorized_keys || "
                f"cat {remote_public} >> ~/.ssh/authorized_keys)")
    print("Cubie authorized_keys updated.")


def upload_and_deploy(client: paramiko.SSHClient, password: str) -> None:
    uploads = {
        ROOT / "tools" / "cubie_optical_flow.cpp": f"{REMOTE_DIR}/cubie_optical_flow.cpp",
        ROOT / "camera1_fisheye_1280x720_rectilinear_f400.yaml":
            f"{REMOTE_DIR}/camera1_fisheye_1280x720_rectilinear_f400.yaml",
        ROOT / "tools" / "cubie-optical-flow.service":
            f"{REMOTE_DIR}/cubie-optical-flow.service",
    }
    sftp = client.open_sftp()
    try:
        for local, remote in uploads.items():
            if not local.is_file():
                raise FileNotFoundError(local)
            sftp.put(str(local), remote + ".new")
    finally:
        sftp.close()
    run(client, f"set -e; cd {REMOTE_DIR}; "
                "g++ -O3 -std=c++17 -pthread -x c++ cubie_optical_flow.cpp.new -o cubie_optical_flow.new "
                "$(pkg-config --cflags --libs opencv4); "
                "mv cubie_optical_flow.cpp.new cubie_optical_flow.cpp; "
                "mv camera1_fisheye_1280x720_rectilinear_f400.yaml.new "
                "camera1_fisheye_1280x720_rectilinear_f400.yaml; "
                "mv cubie_optical_flow.new cubie_optical_flow; "
                "mv cubie-optical-flow.service.new cubie-optical-flow.service")
    run(client,
        "sudo -S -p '' sh -c 'install -m 644 /home/radxa/optical_flow/cubie-optical-flow.service "
        "/etc/systemd/system/cubie-optical-flow.service && systemctl daemon-reload && "
        "systemctl restart cubie-optical-flow && systemctl --no-pager --full status cubie-optical-flow'",
        password + "\n")


def main() -> None:
    args = parse_args()
    if not args.install_key and not args.inspect and not args.deploy:
        raise SystemExit("Specify at least one of --install-key, --inspect, or --deploy")
    password = args.password or getpass.getpass("Cubie SSH password: ")
    client = connect(args, password)
    try:
        if args.install_key:
            install_key(client, args.key)
        if args.inspect:
            print(run(client, "v4l2-ctl -d /dev/video0 --list-ctrls; "
                              "v4l2-ctl -d /dev/video0 --get-fmt-video; "
                              "pkg-config --modversion opencv4; "
                              "systemctl is-active cubie-optical-flow || true"))
        if args.deploy:
            upload_and_deploy(client, password)
    finally:
        client.close()


if __name__ == "__main__":
    main()

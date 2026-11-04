import argparse
import os, time, termios, tty, select, re, json

parser = argparse.ArgumentParser(description="Check the MC02 Log CDC terminal; sends diagnostic commands only.")
parser.add_argument("--port", required=True, help="Log CDC device path (USB interface 00)")
parser.add_argument("--clock", type=int, default=550000000)
parser.add_argument("--output", default="/tmp/wdr_terminal_probe.json")
args = parser.parse_args()
path=args.port
for _ in range(50):
    if os.path.exists(path):break
    time.sleep(.1)
fd=os.open(path,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
tty.setraw(fd)
a=termios.tcgetattr(fd);a[2]|=termios.CLOCAL|termios.CREAD;termios.tcsetattr(fd,termios.TCSANOW,a)
def read_for(seconds):
    end=time.monotonic()+seconds;out=b''
    while time.monotonic()<end:
        if select.select([fd],[],[],max(0,end-time.monotonic()))[0]:out+=os.read(fd,4096)
    return out
read_for(.2)
def command(value):
    os.write(fd,value.encode()+b'\r');out=read_for(.15)
    return re.sub(rb'\x1b\[[0-?]*[ -/]*[@-~]',b'',out).decode(errors='replace')
log=[]
for c in ['', 'ls','status','echo WDR_LOG_731']:
    out=command(c);log.append((c,out));print(repr(out))
assert f'clock_hz={args.clock}' in log[2][1],log
assert log[3][1].count('WDR_LOG_731')>=2,log[3]
for i in range(20):
    token=f'WDR_{i:03}'
    out=command('echo '+token)
    assert out.count(token)>=2,(i,out)
    log.append(('echo '+token,out))
for i in range(3):
    out=command('status');assert f'clock_hz={args.clock}' in out,out;log.append(('status',out))
os.close(fd)
open(args.output,'w').write(json.dumps(log,ensure_ascii=False,indent=2))
print('PASS: command list, board status, 21 echo round trips, repeated status')

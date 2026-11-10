#!/usr/bin/env python3
"""Root service preflight: configure a DOWN CAN interface, never send CAN."""
import json
import subprocess


def main():
    status=json.loads(subprocess.check_output(['ip','-details','-json','link','show','dev','can0']))[0]
    bitrate=status.get('linkinfo',{}).get('info_data',{}).get('bittiming',{}).get('bitrate')
    if 'UP' in status.get('flags',[]):
        if bitrate != 500000:
            raise SystemExit('can0 is already UP at an unexpected bitrate; manual review required')
        return
    subprocess.run(['ip','link','set','can0','type','can','bitrate','500000'],check=True)
    subprocess.run(['ip','link','set','can0','up'],check=True)


if __name__=='__main__':main()

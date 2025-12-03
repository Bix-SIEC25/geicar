# Update Pi4's time
We just curl the [http://time.bix.ovh](http://time.bix.ovh) to get the current timestamp of the [server](https://github.com/Bix-SIEC25/server/blob/main/time/index.php).

Create a script file, `/usr/local/bin/sync_time_from_server.sh`
```bash
#!/usr/bin/env bash
set -euo pipefail

URL='http://time.bix.ovh'
CURL_TIMEOUT=10

# fetch the website
resp=$(curl -fsS --max-time "$CURL_TIMEOUT" "$URL") || {
  echo "ERROR: failed to fetch $URL" >&2
  exit 1
}

# protection against non numeric numbers
ts=${resp//[^0-9]/}

if [[ -z "$ts" ]]; then
  echo "ERROR: no valid timestamp in response: '$resp'" >&2
  exit 2
fi

echo "Setting time to epoch: $ts (UTC)"
date -u -s "@$ts"
#hwclock --systohc

echo "Done."
```
Make it executable
```bash
chmod +x /usr/local/bin/sync_time_from_server.sh
```

Then, create a service `/etc/systemd/system/sync-time.service`
```toml
[Unit]
Description=Sync system clock from server
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/sync_time_from_server.sh

[Install]
WantedBy=multi-user.target
```

And to finally start the service, execute 
```bash
sudo systemctl enable --now sync-time.service
```

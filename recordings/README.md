# Streaming/recording to Rerun

The simulator supports Rerun via the `--rerun` CLI option:

- `none` (default)
- `record`
- `stream`
- `both`

Install dependencies first:

```bash
pip install rerun-sdk rerun-loader-mjcf
```

## CLI flags

- `--rerun {none,record,stream,both}`
- `--rerunfile <name>` (optional)

## Record mode

In `record` or `both` mode, `.rrd` files are written to `rcssservermj/recordings/`.

Filename format:

- default: `YYYY-HH-MM-SS.rrd`
- with `--rerunfile name`: `name_YYYY-HH-MM-SS.rrd`

## Stream mode

In `stream` or `both` mode, the server streams to `127.0.0.1:9876`.

Run the Rerun viewer:

```bash
rerun --bind 127.0.0.1 --port 9876
```

### Remote server (e.g., EC2)

If the simulator runs on a remote machine, open a reverse SSH tunnel from your local machine:

```bash
ssh -i <key-path> -R 9876:localhost:9876 ubuntu@your-ec2-public-dns
```

## Uploading data to Google Drive

To protect local storage space, you can upload generated `.rrd` files to Google Drive using `rclone`.

## `rclone` setup

```bash
curl https://rclone.org/install.sh | sudo bash
rclone config
```

Options to pick, in order, while leaving other things empty:

```text
n) New remote
google_drive
24
1
n) No (default)
n) No (would be Yes if on local)
paste the token
n) No (default)
y) Yes this is OK (default)
q) Quit config
```

Add the shared `rob450-data` folder to My Drive (it must be in the first level of Drive, not in "Shared with me" or in a subfolder, to keep the upload command the same for everybody).

Test by creating a test file and running:

```bash
rclone copy test.txt google_drive:/rob450-data --progress
```

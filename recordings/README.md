# Streaming/recording to Rerun

The command line option `--rerun` has options `{none, record, stream, both}`. Make sure you have installed `rerun-sdk` and `rerun-loader-mjcf` with pip before trying to use any option other than `none` (which is the default).

If you choose to record, the file will be named after the starting time and saved to `rcssservermj/recordings/YYYY-HH-MM-SS.rrd`. To change the name of this file, pass in the desired name to the `--rerun-file` command-line argument. Do not include the .rrd extension. The timestamp will still be appended after the desired filename. 

If you choose to stream, the server will stream on 127.0.0.1:9876. Run this command to watch:
```rerun --bind 127.0.0.1 --port 9876```

If you are streaming from an AWS EC2, also run this command locally to watch:
```ssh -i <key-path> -R 9876:localhost:9876 ubuntu@your-ec2-public-dns```

If you are streaming from CAEN, run this command locally to watch:
```ssh -R 9876:localhost:9876 uniqname@login.engin.umich.edu```

# Uploading data to Google Drive

To protect local storage space, you can also upload the files to Google Drive. Add the command-line option `--upload` to do so. This requires `rclone` to be set up.

## `rclone` setup

```
curl https://rclone.org/install.sh | sudo bash
rclone config
```

Options to pick, in order, while leaving other things empty:

```
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

Add the shared rob450-data folder to My Drive (it must be in the first level of Drive, not in shared with me or in a subfolder, to keep the upload command the same for everybody).

Test by creating a test.txt file and running this to see if it shows up in the shared data foler: 

```rclone copy test.txt google_drive:/rob450-data --progress```

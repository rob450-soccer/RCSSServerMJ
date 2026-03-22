# Streaming/recording to Rerun

The command line option `--rerun` has options `{none, record, stream, both}`. Make sure you have installed `rerun-sdk` and `rerun-loader-mjcf` with pip before trying to use any option other than `none` (which is the default).

If you choose to record, the file will be named after the starting time and saved to `rcssservermj/recordings/YYYY-HH-MM-SS.rrd`. 

If you choose to stream, the server will stream on 127.0.0.1:9876. Run this command to watch:
```rerun --bind 127.0.0.1 --port 9876```

If you are streaming from an AWS EC2, also run this command locally to watch:
```ssh -i <key-path> -R 9876:localhost:9876 ubuntu@your-ec2-public-dns```

If you are streaming from CAEN, run this command locally to watch:
```ssh -R 9876:localhost:9876 uniqname@login.engin.umich.edu```

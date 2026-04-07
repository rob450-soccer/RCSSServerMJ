"""
Rerun adapter

Instructions can be found in rcssservermj/recordings/README.md
"""

import logging
import datetime
from pathlib import Path
from rcsssmj.sim.simulation import BaseSimulation


sim_logger = logging.getLogger(__name__)


# see if Rerun is set up
SETUP = None
try:
    import rerun as rr
    import rerun_loader_mjcf

    SETUP = True
except ImportError:
    sim_logger.warning('Rerun is not set up. Recording and streaming will not work unless it is set up.')
    SETUP = False


class RerunAdapter:
    def __init__(self, mode, sim: BaseSimulation, file: str | None = None):
        """Set up recording or streaming to Rerun."""
        global SETUP
        self.mode = mode
        self.sim = sim
        self.rr_logger = None
        self.recorder = None
        self._record_path = None
        self._last_timestamp = float('-inf')

        # no-op if Rerun is not enabled
        if self.mode == 'none':
            return
        if not SETUP:
            sim_logger.error("Rerun was requested but could not be set up.")

        try: 
            rr.init('rcssservermj', spawn=False, default_enabled=True)

            # set the record path
            current_dir = Path(__file__).resolve().parent
            project_root = current_dir.parents[2]
            timestamp = datetime.datetime.now().strftime("%Y-%H-%M-%S")
            self._record_path = project_root / "recordings" / f"{file + '_' if file != 'none' else ''}{timestamp}.rrd"
            self._record_path.parent.mkdir(parents=True, exist_ok=True)
            self._record_path = str(self._record_path)

            self._configure_sinks()

            self.rr_logger = rerun_loader_mjcf.MJCFLogger(self.sim.mj_model)
            rr.set_time('sim_time', timestamp=0.0)
            self.rr_logger.log_model()

            self.recorder = rerun_loader_mjcf.MJCFRecorder(self.rr_logger, timeline_name="sim_time")

            sim_logger.info(f"Rerun successfully set up in {self.mode} mode.")
        
        except Exception as e:
            sim_logger.error(f"Rerun was requested but could not be set up due to an error: {e}")
            SETUP = False

    def _configure_sinks(self):
        """Configure sinks for the current adapter mode."""
        if self.mode == "record":
            rr.save(self._record_path)
            sim_logger.info(f"Rerun saving recording to {self._record_path}")
        elif self.mode == "stream":
            rr.set_sinks(rr.GrpcSink("rerun+http://127.0.0.1:9876/proxy"))
            sim_logger.info("Rerun streaming to localhost:9876. Run `rerun --bind 127.0.0.1 --port 9876` to watch.")
        elif self.mode == 'both':
            rr.set_sinks(rr.GrpcSink("rerun+http://127.0.0.1:9876/proxy"), rr.FileSink(self._record_path))
            sim_logger.info(f"Rerun streaming to localhost:9876 and saving to {self._record_path}. ")

    def _get_monotonic_timestamp(self, timestamp: float) -> float:
        """Ensure timestamps never go backwards for Rerun timelines."""
        ts = float(timestamp)
        if ts <= self._last_timestamp:
            ts = self._last_timestamp + 1e-6
        self._last_timestamp = ts
        return ts

    
    def recompile(self, data, model):
        """Recompile the logger and recorder when the world data is recompiled."""
        global SETUP
        # no-op if Rerun is not enabled
        if not SETUP or self.mode == 'none':
            return

        try:
            ts = self._get_monotonic_timestamp(data.time)
            self.rr_logger = rerun_loader_mjcf.MJCFLogger(model)
            self.rr_logger.log_model()
            self.recorder = rerun_loader_mjcf.MJCFRecorder(self.rr_logger, timeline_name="sim_time")
            rr.set_time('sim_time', timestamp=ts)
            self.recorder.record(data, timestamp=ts)
            self.recorder.flush()
        except Exception as e:
            sim_logger.warning(f"There was an error with Rerun, turning it off. Error: {e}")
            SETUP = False


    def step(self, data, timestamp):
        """Record/stream a single step of the simulation."""
        global SETUP
        # no-op if Rerun is not enabled
        if not SETUP or self.mode == 'none':
            return

        try:
            ts = self._get_monotonic_timestamp(timestamp)
            rr.set_time('sim_time', timestamp=ts)
            self.recorder.record(data, timestamp=ts)
            self.recorder.flush()
        except Exception as e:
            sim_logger.warning(f"There was an error with Rerun, turning it off. Error: {e}")
            SETUP = False

    def shutdown(self):
        """Shutdown Rerun if it was active."""
        global SETUP
        # no-op if Rerun is not enabled
        if not SETUP or self.mode == 'none':
            return

        if self.recorder is not None:
            try:
                self.recorder.flush()
                rr.flush()
            except Exception:
                pass
        
        sim_logger.info("Rerun shut down.")

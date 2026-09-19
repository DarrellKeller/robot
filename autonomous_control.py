"""Default entry point for the Jev controller. Legacy code uses the old firmware."""
import sys

if __name__ == "__main__":
    if "--legacy" in sys.argv:
        import runpy
        from pathlib import Path
        sys.argv.remove("--legacy")
        runpy.run_path(str(Path(__file__).with_name("legacy_autonomous_control.py")), run_name="__main__")
    else:
        from jev_control import main
        main()

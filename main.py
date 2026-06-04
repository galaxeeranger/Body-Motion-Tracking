# from calib.pipeline import CalibPipeline

# if __name__ == "__main__":
#     CalibPipeline().run()
from calib.pipeline import CalibPipeline
from viz.app        import run_app

if __name__ == "__main__":
    pipeline = CalibPipeline()
    run_app(pipeline)
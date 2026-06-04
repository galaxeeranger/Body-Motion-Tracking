# import sys
# from PyQt5.QtWidgets import QApplication
# from PyQt5.QtCore    import QThread, pyqtSignal, QObject


# # ────────────────────────────────────────────────────────
# # Worker — runs pipeline.run() in background thread
# # ────────────────────────────────────────────────────────
# class PipelineWorker(QObject):
#     finished = pyqtSignal()
#     error    = pyqtSignal(str)

#     def __init__(self, pipeline):
#         super().__init__()
#         self._pipeline = pipeline

#     def run(self):
#         try:
#             self._pipeline.run()
#             self.finished.emit()
#         except Exception as e:
#             self.error.emit(str(e))


# # ────────────────────────────────────────────────────────
# # App controller — owns QThread lifecycle
# # ────────────────────────────────────────────────────────
# class AppController(QObject):
#     """
#     Manages pipeline thread.
#     Exposes restart_pipeline() so sidebar button can call it.
#     """

#     def __init__(self, pipeline):
#         super().__init__()
#         self._pipeline = pipeline
#         self._thread   = None
#         self._worker   = None

#     def start_pipeline(self):
#         """Starts pipeline.run() in a fresh QThread."""
#         self._thread = QThread()
#         self._worker = PipelineWorker(self._pipeline)
#         self._worker.moveToThread(self._thread)

#         self._thread.started.connect(self._worker.run)
#         self._worker.finished.connect(self._on_finished)
#         self._worker.finished.connect(self._thread.quit)
#         self._worker.finished.connect(self._worker.deleteLater)
#         self._worker.error.connect(self._on_error)
#         self._worker.error.connect(self._thread.quit)
#         self._thread.finished.connect(self._thread.deleteLater)

#         self._thread.start()

#     def restart_pipeline(self):
#         """
#         Called by recalibrate button.
#         1. Stops current thread safely
#         2. Resets pipeline + state
#         3. Starts fresh thread
#         """
#         # step 1 — stop current thread if running
#         if self._thread and self._thread.isRunning():
#             self._thread.quit()
#             self._thread.wait(5000)   # wait max 5 sec

#         # step 2 — reset pipeline (state + stages)
#         self._pipeline.reset()

#         # step 3 — start fresh
#         self.start_pipeline()

#     def _on_finished(self):
#         print("✅ Pipeline finished normally.")

#     def _on_error(self, msg):
#         print(f"❌ Pipeline error: {msg}")

#     def stop(self):
#         """Called on window close."""
#         if self._thread and self._thread.isRunning():
#             self._thread.quit()
#             self._thread.wait(3000)


# # ────────────────────────────────────────────────────────
# # Entry point — called from main.py
# # ────────────────────────────────────────────────────────
# def run_app(pipeline):
#     """
#     1. Creates QApplication
#     2. Creates AppController (owns thread)
#     3. Shows MainWindow
#     4. Starts pipeline thread
#     5. Blocks on app.exec_()
#     """
#     app = QApplication.instance() or QApplication(sys.argv)
#     app.setStyle("Fusion")

#     # import here to avoid circular imports
#     from viz.window import MainWindow

#     # create controller first
#     controller = AppController(pipeline)

#     # create window — pass controller so it can wire recalibrate button
#     window = MainWindow(pipeline.state, controller)
#     window.show()

#     # start pipeline in background
#     controller.start_pipeline()

#     # block until window closed
#     exit_code = app.exec_()

#     # clean stop
#     controller.stop()
#     sys.exit(exit_code)
import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore    import QThread, pyqtSignal, QObject


# ────────────────────────────────────────────────────────
# Worker — runs pipeline.run() in background thread
# ────────────────────────────────────────────────────────
class PipelineWorker(QObject):
    finished = pyqtSignal()
    error    = pyqtSignal(str)

    def __init__(self, pipeline):
        super().__init__()
        self._pipeline = pipeline

    def run(self):
        try:
            self._pipeline.run()
            self.finished.emit()
        except Exception as e:
            self.error.emit(str(e))


# ────────────────────────────────────────────────────────
# App controller — owns QThread lifecycle
# ────────────────────────────────────────────────────────
class AppController(QObject):
    """
    Manages pipeline thread.
    Exposes restart_pipeline() so sidebar button can call it.
    """

    def __init__(self, pipeline):
        super().__init__()
        self._pipeline = pipeline
        self._thread   = None
        self._worker   = None

    def start_pipeline(self):
        """Starts pipeline.run() in a fresh QThread."""
        self._thread = QThread()
        self._worker = PipelineWorker(self._pipeline)
        self._worker.moveToThread(self._thread)

        self._thread.started.connect(self._worker.run)
        self._worker.finished.connect(self._on_finished)
        self._worker.finished.connect(self._thread.quit)
        self._worker.finished.connect(self._worker.deleteLater)
        self._worker.error.connect(self._on_error)
        self._worker.error.connect(self._thread.quit)

        # ── fix: clear reference after deletion so isRunning() is never
        #         called on a deleted C++ object ──
        self._thread.finished.connect(self._thread.deleteLater)
        self._thread.finished.connect(self._clear_thread)

        self._thread.start()

    def _clear_thread(self):
        """Called after thread fully finishes. Prevents deleted object access."""
        self._thread = None
        self._worker = None

    def restart_pipeline(self):
        """
        Called by recalibrate button.
        1. Stops current thread safely
        2. Resets pipeline + state
        3. Starts fresh thread
        """
        # step 1 — check None first, then isRunning()
        if self._thread is not None and self._thread.isRunning():
            self._thread.quit()
            self._thread.wait(5000)   # wait max 5 sec

        # step 2 — reset pipeline (state + stages)
        self._pipeline.reset()

        # step 3 — start fresh
        self.start_pipeline()

    def _on_finished(self):
        print("✅ Pipeline finished normally.")

    def _on_error(self, msg):
        print(f"❌ Pipeline error: {msg}")

    def stop(self):
        """Called on window close."""
        if self._thread is not None and self._thread.isRunning():
            self._thread.quit()
            self._thread.wait(3000)


# ────────────────────────────────────────────────────────
# Entry point — called from main.py
# ────────────────────────────────────────────────────────
def run_app(pipeline):
    """
    1. Creates QApplication
    2. Creates AppController (owns thread)
    3. Shows MainWindow
    4. Starts pipeline thread
    5. Blocks on app.exec_()
    """
    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyle("Fusion")

    # import here to avoid circular imports
    from viz.window import MainWindow

    # create controller first
    controller = AppController(pipeline)

    # create window — pass controller so it can wire recalibrate button
    window = MainWindow(pipeline.state, controller)
    window.show()

    # start pipeline in background
    controller.start_pipeline()

    # block until window closed
    exit_code = app.exec_()

    # clean stop
    controller.stop()
    sys.exit(exit_code)
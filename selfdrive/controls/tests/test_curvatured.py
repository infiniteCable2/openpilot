import cereal.messaging as messaging

from openpilot.selfdrive.controls.lib.curvatured import CurvatureDController, CurvatureDLookup, VERSION


class TestCurvatureDController:

  def test_apply_uses_matching_bucket_only(self):
    controller = CurvatureDController()
    msg = messaging.new_message('liveCurvatureParameters')
    msg.liveCurvatureParameters.liveValid = True
    msg.liveCurvatureParameters.version = VERSION
    msg.liveCurvatureParameters.useParams = True

    idx = CurvatureDLookup.indices(32e-6, 22.0)
    assert idx is not None
    msg.liveCurvatureParameters.currentCorrection = 20e-6
    msg.liveCurvatureParameters.currentBias = 20e-6
    msg.liveCurvatureParameters.currentBucketPoints = CurvatureDLookup.FULL_CONFIDENCE_SAMPLES
    msg.liveCurvatureParameters.bucketSign = idx[0]
    msg.liveCurvatureParameters.bucketSpeed = idx[1]

    controller.update_live_params(msg.liveCurvatureParameters)

    assert controller.apply(32e-6, 22.0) > 32e-6
    assert controller.apply(32e-6, 38.0) == 32e-6
    assert controller.apply(2e-4, 22.0) == 2e-4

  def test_invalid_message_disables_corrections(self):
    controller = CurvatureDController()
    msg = messaging.new_message('liveCurvatureParameters')
    msg.liveCurvatureParameters.liveValid = False
    msg.liveCurvatureParameters.version = VERSION
    msg.liveCurvatureParameters.useParams = True
    msg.liveCurvatureParameters.bucketSign = -1
    msg.liveCurvatureParameters.bucketSpeed = -1

    controller.update_live_params(msg.liveCurvatureParameters)

    assert controller.apply(32e-6, 20.0) == 32e-6

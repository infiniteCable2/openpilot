import cereal.messaging as messaging

from openpilot.selfdrive.controls.lib.curvatured import CurvatureDController, CurvatureDLookup, VERSION


class TestCurvatureDController:

  def test_apply_uses_matching_bucket_only(self):
    controller = CurvatureDController()
    msg = messaging.new_message('liveCurvatureParameters')
    msg.liveCurvatureParameters.liveValid = True
    msg.liveCurvatureParameters.version = VERSION
    msg.liveCurvatureParameters.useParams = True

    bias = CurvatureDLookup.unflatten([0.0] * CurvatureDLookup.total_size())
    counts = CurvatureDLookup.unflatten([0] * CurvatureDLookup.total_size())

    idx = CurvatureDLookup.indices(64e-6, 22.0)
    assert idx is not None
    bias[idx] = 20e-6
    counts[idx] = CurvatureDLookup.FULL_CONFIDENCE_SAMPLES

    msg.liveCurvatureParameters.bias = CurvatureDLookup.flatten(bias)
    msg.liveCurvatureParameters.counts = CurvatureDLookup.flatten(counts)

    controller.update_live_params(msg.liveCurvatureParameters)

    assert controller.apply(64e-6, 22.0) > 64e-6
    assert controller.apply(64e-6, 38.0) == 64e-6

  def test_invalid_message_disables_corrections(self):
    controller = CurvatureDController()
    msg = messaging.new_message('liveCurvatureParameters')
    msg.liveCurvatureParameters.liveValid = False
    msg.liveCurvatureParameters.version = VERSION
    msg.liveCurvatureParameters.useParams = True
    msg.liveCurvatureParameters.bias = []
    msg.liveCurvatureParameters.counts = []

    controller.update_live_params(msg.liveCurvatureParameters)

    assert controller.apply(32e-6, 20.0) == 32e-6

build_lib(
  LIBNAME lr-wpan
  SOURCE_FILES
    helper/lr-wpan-helper.cc
    model/lr-wpan-error-model.cc
    model/lr-wpan-fields.cc
    model/lr-wpan-interference-helper.cc
    model/lr-wpan-lqi-tag.cc
    model/lr-wpan-mac-header.cc
    model/lr-wpan-mac-pl-headers.cc
    model/lr-wpan-mac-trailer.cc
    model/lr-wpan-mac-base.cc
    model/lr-wpan-mac.cc
    model/lr-wpan-net-device.cc
    model/lr-wpan-phy.cc
    model/lr-wpan-spectrum-signal-parameters.cc
    model/lr-wpan-spectrum-value-helper.cc

    model/lr-wpan-csmaca.cc
    model/lr-wpan-csmaca-noba.cc
    model/lr-wpan-csmaca-sw-noba.cc
    model/lr-wpan-csmaca-gnu-noba.cc
    model/lr-wpan-csmaca-standard.cc
    model/lr-wpan-csmaca-common.cc
    model/lr-wpan-delay-tag.cc
    model/lr-wpan-priority-tag.cc
    model/lr-wpan-retransmission-tag.cc

  HEADER_FILES
    helper/lr-wpan-helper.h
    model/lr-wpan-constants.h
    model/lr-wpan-error-model.h
    model/lr-wpan-fields.h
    model/lr-wpan-interference-helper.h
    model/lr-wpan-lqi-tag.h
    model/lr-wpan-mac-header.h
    model/lr-wpan-mac-pl-headers.h
    model/lr-wpan-mac-trailer.h
    model/lr-wpan-mac-base.h
    model/lr-wpan-mac.h
    model/lr-wpan-net-device.h
    model/lr-wpan-phy.h
    model/lr-wpan-spectrum-signal-parameters.h
    model/lr-wpan-spectrum-value-helper.h

    model/lr-wpan-csmaca.h
    model/lr-wpan-csmaca-noba.h
    model/lr-wpan-csmaca-sw-noba.h
    model/lr-wpan-csmaca-gnu-noba.h
    model/lr-wpan-csmaca-standard.h
    model/lr-wpan-csmaca-common.h
    model/lr-wpan-delay-tag.h
    model/lr-wpan-priority-tag.h
    model/lr-wpan-retransmission-tag.h


  LIBRARIES_TO_LINK ${libspectrum}
  TEST_SOURCES
    test/lr-wpan-ack-test.cc
    test/lr-wpan-cca-test.cc
    test/lr-wpan-collision-test.cc
    test/lr-wpan-ed-test.cc
    test/lr-wpan-error-model-test.cc
    test/lr-wpan-packet-test.cc
    test/lr-wpan-pd-plme-sap-test.cc
    test/lr-wpan-spectrum-value-helper-test.cc
    test/lr-wpan-ifs-test.cc
    test/lr-wpan-slotted-csmaca-test.cc
    test/lr-wpan-mac-test.cc
)

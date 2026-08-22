idf_component_register(
    SRCS
        "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        audio_pipeline
        audio_stream
        bt
        driver
        bluetooth_service
)

import os

import numpy as np
import tensorrt as trt


TRT_LOGGER = trt.Logger()

# This script converts an ONNX export to TensorRT, which lets you use the DLNA accelerator cores on your device
# This has to be run on the device that the inference will occur on to get the best actual performance

onnx_file_path = "/home/robot/yolov5s.onnx"
engine_file_path = "/home/robot/yolov5s.trt"

with trt.Builder(TRT_LOGGER) as builder, builder.create_network() as network, trt.OnnxParser(network, TRT_LOGGER) as parser:
    builder.max_workspace_size = 1 << 30 # 1GB
    builder.max_batch_size = 1

    # Parse model file
    if not os.path.exists(onnx_file_path):
        print('ONNX file {} not found, please run yolov3_to_onnx.py first to generate it.'.format(onnx_file_path))
        exit(0)

    print('Loading ONNX file from path {}...'.format(onnx_file_path))
    with open(onnx_file_path, 'rb') as model:
        print('Beginning ONNX file parsing')
        parser.parse(model.read())
    print('Completed parsing of ONNX file')
    print('Building an engine from file {}; this may take a while...'.format(onnx_file_path))
    engine = builder.build_cuda_engine(network)
    print("Completed creating Engine")
    with open(engine_file_path, "wb") as f:
        f.write(engine.serialize())
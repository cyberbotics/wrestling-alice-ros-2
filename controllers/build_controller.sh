#!/bin/bash

# Build the "builder" image that will build the controller files
docker build -f Build_Dockerfile -t builder_container .
# Copy the files to the builder shared volume
docker run -t -v "$(pwd)"/built_controller/:/usr/local/webots-project/controllers/participant/built_controller:rw builder_container \
	cp -r install built_controller
# Recover the built files and copy them to the correct spot
cp -r built_controller/install participant


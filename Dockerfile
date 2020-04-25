FROM johannhaselberger/coros:latest
RUN pwd
WORKDIR /my_ros_data
RUN pwd
ADD setup.sh .
ADD runonce.sh .
RUN pwd
RUN ls

# rs_type_adapter

This package is meant to show off the performance benefits of using type adaptation when transferring data between the realsenses camera and some ROS subscribers. This examples were made on a Jetson AGX Xavier with a Intel Realsense <model> connected. We used the realsesnse-ros driver and modified it in order to use type adaptation for this examples. 

## Compilation

Compile like any ROS 2 package.
This has been tested on ROS 2 Humble.

## Test setup

For the rest of the discussion, we'll be assuming a setup with a publisher (the realsense camera) and one or multiple subscribers to the camera topic.

The subscriber is accepting the message, and printing the memory direction of the image.

## Example 1: Separate processes, no type adaptation, no intra-process

Run the following command:

```
ros2 launch rs_type_adapter_example image_no_type_adapt_no_intra.py
```

In this example, we launch two separate processes, one for the publisher and one for the subscriber.
For every image that is transmitted, roughly the following steps happen:

1.  The publisher allocates a `sensor_msgs::msg::Image` message, along with space for the data.
2.  The publisher fills out the data for the message.
3.  The publisher publishes the message, which ends up calling through the RMW stack, serializing the data, and delivering the data via UDP over localhost.
4.  The RMW layer on the subscriber receives the data via UDP over localhost, deserializes the data, and delivers it to the callback.

As can be seen, there is a lot of serializing, deserializing, and copying of data going on to deliver the data from one process to the next.
The result of this example on the setup described above is that the publisher takes (Missing) of a CPU to deliver the data, while the subscriber takes (Missing) to receive the data.

## Example 2: Single process composed, no type adaptation, intra-process enabled
Let's try to improve the results by composing eveything into a single process and use the intra-process communication.

This example can be run with the following launch file:

```
ros2 launch rs_type_adapter_example image_no_type_adapt.py
```

Indeed, by running the processes like this, we recognize a bunch of savings; the same data takes only ~22% CPU to both send and receive.

## Example 3: Single process composed, using type adaptation, intra-process enabled

Now let's try to improve the results even further by using a type adapter to send the image.

This example can be run with the following launch file:

```
ros2 launch type_adapt_example image_type_adapt_intra-composed-launch.py
```

When we run this example, it takes ~20% CPU to send and receive the data within the process.

We can see that the improvement is not something that relevant between the use of type adaptation and just using the intra-process communication. However, let's take a look at what happens when there is not just one subscriber but multiple

## Example 4: Single process composed, multiple subscribers, no type adaptation, intra-process enabled

This example can be run with the following launch file:

```
ros2 launch rs_type_adapter_example image_multiple_no_type_adapt.py
```

When we run this example, we can see that the CPU usage goes up to (Missing %) CPU to send and receive the data within the process. This is happening because of the multiple copies being made by the subscribers, so this means that if we keep adding subscribers to this same topic, the CPU usage will keep increasing. Let's see what happens when we use multiple subscribers with type adaptation.

## Example 5: Single process composed, multiple subscribers, using type adaptation, intra-process enabled

This example can be run with the following launch file:

```
ros2 launch rs_type_adapter_example image_multiple_no_type_adapt.py
```

When we run this example, we can see that the usage is around ~20% CPU, just like in the Example 3, to send and receive the data within the process. This is happening because all the subscribers are using the exact same reference of the image container which allow us to reduce the computational cost since there are not multiple copies being created, so this means that if we keep adding subscribers to this same topic, the CPU usage will stay the same as long as the subscribers doesn't need to modify the image. 

Put rviz_ws.tar.gz here (produced on the Jetson by the rviz-builder container,
see the "RViz2 build" note in the README) to make the web image build extract
the prebuilt RViz workspace instead of recompiling ~2.5h from source.
The from-source recipe stays in web.Dockerfile as the fallback.

    docker cp rviz-builder:/opt/rviz_ws - | tar -czf rviz_ws.tar.gz -C /dev/stdin rviz_ws
    # (then place the tar at docker/prebuilt/rviz_ws.tar.gz on the deploy machine)

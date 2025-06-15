# Turtle Nest Tests

## How to run

1. Open the docker container (from the repository root)

    ```
    cd docker
    docker compose up -d
    docker exec -it turtle_nest bash
    ```

1. run:
   ```
   cd /home/user/ros2_ws/src/turtle_nest_tests/build/
   make
   ./turtle_nest_tests
   ```
   
Optionally, you can filter tests with:

```
./turtle_nest_tests --gtest_filter=<TestSuiteName>.<TestName>
```


## Code formatting

Tests in the CI also run colcon-test, which confirms that the code formatting is as expected.
To apply the code formatting automatically, run

```commandline
ament_uncrustify --reformat
```

for the turtle_nest package.

Python3 files can be auto-formatted with:
```commandline
python3 -m autopep8 --in-place --aggressive --aggressive <file>
python3 -m docformatter --in-place --wrap-summaries 88 --wrap-descriptions 88 <file>
```
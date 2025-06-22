from test_example_msgs.msg import Example

def main():
    msg = Example()
    msg.example_data = "example text"
    print(msg)

if __name__ == "__main__":
    main()
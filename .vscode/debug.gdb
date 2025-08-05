    # Connect to the remote target
    target extended-remote :1337
    # Load the application
    load
    # Set a breakpoint on our entry to main
    break main
    # Start execution
    continue

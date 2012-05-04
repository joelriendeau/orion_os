NOTE : This library uses boost and std facilities which require dynamic memory allocation, and RTTI. The dynamic memory allocation can
       easily be provided by a memory pool implementation on new() and delete(), but the RTTI needs actual compiler support.

This library fulfills many purposes related to storing settings and configuration in files and in memory.

- It can create / load config files.
- Configurations are defined by external classes so software can support many versions of a config file, and convert between versions to support upgrading.
- It supports defining default values for settings which are not defined.
- It supports storing the settings in memory for runtime queries, and serves as the central repository of those settings.

* All custom types that needs to be defined (enums, structs, etc.) must be serializable to text using a conversion you provide. Standard types
  are supported out of the box (bools, ints, doubles, etc.) through boost::lexical_cast

This library should be kept highly portable, as it WILL be used onboard to store ephemeris data and on the PDA to store user settings. It should
NOT provide output or input to files as they are not garanteed to be available on all platforms; should IO to memory buffers instead,
the target platform will handle reading / writing to storage.
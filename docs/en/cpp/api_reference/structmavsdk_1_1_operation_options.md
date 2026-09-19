# mavsdk::OperationOptions Struct Reference
`#include: operation_options.hpp`

----


Options shared by blocking MAVSDK operations. 


## Data Fields


std::chrono::milliseconds [timeout](#structmavsdk_1_1_operation_options_1a04247272efdf8beb3977e9e19d231bda) {} - Maximum time for the complete operation, including queueing and retries.


## Field Documentation


### timeout {#structmavsdk_1_1_operation_options_1a04247272efdf8beb3977e9e19d231bda}

```cpp
std::chrono::milliseconds mavsdk::OperationOptions::timeout {}
```


Maximum time for the complete operation, including queueing and retries.

A timeout must be greater than zero. The operation returns its existing timeout result when this budget expires.
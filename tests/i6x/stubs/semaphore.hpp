#pragma once
namespace LibXR
{
struct Semaphore { explicit Semaphore(int) {} };
struct ReadOperation { ReadOperation(Semaphore&, int) {} };
}

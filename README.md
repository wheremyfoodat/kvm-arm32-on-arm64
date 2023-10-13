# What is this?
A minimal example showing how to get a virtual machine running arm32 code on an arm64 host using the Linux KVM API. In other words, this creates a virtual machine capable of running arm32 code natively on arm64 CPUs that support hardware virtualization and arm32 backwards-compatibility mode. This is useful for efficient, low-overhead emulation of arm32 systems, since we can run the code directly without needing to do any recompilation, which would result in overhead, both due to compilation times and due to the codegen not being perfect.

# How?
This demo does the bare minimum required, as it's a minimal example and I couldn't find any other aa32-on-aa64 virtualization examples online. It uses `/dev/kvm` to create a virtual machine with a single vCPU attached to it. The vCPU is a regular ARMv8 vCPU configured to start in 32-bit mode, with support for PSCI v0.2 for power control (turning the CPU on/off, suspending it, and so on).

2 pieces of memory are allocated for our VM: A RAM region that's read/write and contains the program code, and another read-only region called the "Hypervisor IO region" which is abused for making hypervisor requests via data aborts caused by writing to it.

The vCPU can run one of 2 tiny programs included in the source code; the first one demonstrates using HVCs by using a PCSI HVC to stop the CPU, and the other, default program shows how to perform a simple hypervisor exit by writing to the aforementioned hypervisor IO area.

A hypervisor would also usually do more stuff, such as setting up interrupts and the GIC (Global Interrupt Controller), setting up the MMU, setting up a userland (called EL0 on arm64) for guest code to run, and so on. This is omitted in this example for the sake of simplicity, otherwise the code would become really complex and hard to follow, defeating the point of this being a simple example.

# Building
The example comes in the form of a single source file. To build it just navigate to the `src` folder and do `g++ main.cpp -o main.out` (or clang++ if using clang)
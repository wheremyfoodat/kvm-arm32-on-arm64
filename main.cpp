#include <fcntl.h>
#include <linux/kvm.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using u32 = std::uint32_t;
using u64 = std::uint64_t;

constexpr u32 LOAD_ADDR = 0xBFC00000;           // Address to load our code to
constexpr u32 HYPERVISOR_IO_ADDR = 0xDDD00000;  // Base address for hypervisor IO. This example writes to this read-only area to trigger hv exits

namespace KVM {
	struct VirtualMachine {
		int kvmDescriptor;  // File descriptor for /dev/kvm
		int fd;             // File descriptor for the VM
		int currentMemorySlot = 0;

		void* ram = nullptr;
		void* hypervisorIO = nullptr;

		VirtualMachine();
	};

	class VirtualCPU {
		int fd = -1;
		kvm_run* runInfo = nullptr;       // Using the KVM_RUN ioctl will output info here
		kvm_reg_list* regList = nullptr;  // List of registers accessible via KVM_{G/S}ET_ONE_REG

		std::array<u64, 32> gprIDs = {};  // IDs for GPRs, to be used with KVM_{G/S}ET_ONE_REG
		void setupRegisterIndices();

	  public:
		bool init(const VirtualMachine& vm);  // Returns true on success
		void run();

		u32 getGPR(int index);
		void setGPR(int index, u32 value);

		u32 getPstate();
		void setPstate(u32 value);

		u32 getPC();
		void setPC(u32 value);
	};
}  // namespace KVM

KVM::VirtualMachine::VirtualMachine() : currentMemorySlot(0) {
	// Open a file descriptor for /dev/kvm. We will use this fd to set up and operate our vCPU using ioctl calls
	kvmDescriptor = open("/dev/kvm", O_RDWR);
	if (kvmDescriptor < 0) {
		printf("Failed to initialize KVM file descriptor\n");
		return;
	}

	// Open a file descriptor for the vCPU
	fd = ioctl(kvmDescriptor, KVM_CREATE_VM, 0);
	if (fd < 0) {
		printf("Failed to initialize create KVM virtual machine\n");
		return;
	}

	// Set up dummy VM memory for testing
	constexpr u64 ramSize = 64 * 1024;
	ram = mmap(NULL, ramSize, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);

	// Map 64KB chunk at address 0
	kvm_userspace_memory_region memory;
	memory.slot = currentMemorySlot++;
	memory.flags = 0;
	memory.guest_phys_addr = LOAD_ADDR;
	memory.memory_size = ramSize;
	memory.userspace_addr = (u64)ram;

	u32* code = (u32*)ram;
	u32 index = 0;

	/*
	// Exit way #1:
	// Set r0 to PSCI SYSTEM_OFF command and execute a hypervisor call. Clobbers registers, and, well, sends an HVC which may be unwanted

	code[index++] = 0xE3A00321; // mov r0, #0x84000000
	code[index++] = 0xE3800008; // orr r0, #8
	code[index++] = 0xE3A06069; // mov r6, #0x69
	code[index++] = 0xE1400070; // hvc #0
	*/

	// Exit way #2, making a quick vmexit by writing to our reserved hypervisor IO area which is located at 0xDDD0'0000-0xDDD0'0FFF
	// Only requires clobbering 1 register for the pointer. When the write happens, KVM_RUN will exit and the hypervisor can handle the request
	code[index++] = 0xE3A004DD;  // mov r0, #0xDD000000
	code[index++] = 0xE380060D;  // orr r0, #0x00D00000
	code[index++] = 0xE3A06069;  // mov r6, #0x69
	code[index++] = 0xE5800000;  // str r0, [r0]

	if (ioctl(fd, KVM_SET_USER_MEMORY_REGION, &memory) < 0) {
		printf("Failed to set up vCPU memory\n");
		return;
	}

	hypervisorIO = mmap(NULL, 0x1000, PROT_READ, MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
	kvm_userspace_memory_region hypervisorIORegion;
	memory.slot = currentMemorySlot++;
	memory.flags = KVM_MEM_READONLY;  // We want writes here to cause VM exits
	memory.guest_phys_addr = HYPERVISOR_IO_ADDR;
	memory.memory_size = 0x1000;
	memory.userspace_addr = (u64)hypervisorIO;

	if (ioctl(fd, KVM_SET_USER_MEMORY_REGION, &hypervisorIORegion) < 0) {
		printf("Failed to set up hypervisor IO memory region\n");
		return;
	}

	printf("Successfully created VM\n");
}

#define AARCH64_CORE_REG(x) (KVM_REG_ARM64 | KVM_REG_SIZE_U64 | KVM_REG_ARM_CORE | KVM_REG_ARM_CORE_REG(x))
#define AARCH64_SIMD_CORE_REG(x) (KVM_REG_ARM64 | KVM_REG_SIZE_U128 | KVM_REG_ARM_CORE | KVM_REG_ARM_CORE_REG(x))
#define AARCH64_SIMD_CTRL_REG(x) (KVM_REG_ARM64 | KVM_REG_SIZE_U32 | KVM_REG_ARM_CORE | KVM_REG_ARM_CORE_REG(x))

bool KVM::VirtualCPU::init(const KVM::VirtualMachine& vm) {
	// Check that this CPU supports aa32-on-aa64 mode, otherwise we can't use KVM on 64-bit ARM to emulate the 3DS
	if (ioctl(vm.fd, KVM_CHECK_EXTENSION, KVM_CAP_ARM_EL1_32BIT) <= 0) {
		printf("CPU doesn't support AA32 mode, KVM won't work on this CPU\n");
		return false;
	}

	// Actually create vCPU
	fd = ioctl(vm.fd, KVM_CREATE_VCPU, 0);
	if (fd < 0) {
		printf("Failed to create KVM vCPU\n");
		return false;
	}

	// Get size of memory used for the kvm_run structure and allocate it via mmap
	int mmapSize = ioctl(vm.kvmDescriptor, KVM_GET_VCPU_MMAP_SIZE, 0);
	if (mmapSize < 0) {
		printf("Failed to get KVM shared memory size\n");
		return false;
	}

	runInfo = (kvm_run*)mmap(nullptr, mmapSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (runInfo == MAP_FAILED) {
		printf("Failed to map KVM shared memory\n");
		return false;
	}

	// Get ideal initialization parameters for the vCPU to feed to KVM_ARM_VCPU_INIT
	kvm_vcpu_init initParams;
	if (ioctl(vm.fd, KVM_ARM_PREFERRED_TARGET, &initParams) < 0) {
		printf("Failed to fetch initialization parameters for vCPU\n");
		return false;
	}

	// Request 32-bit vCPU with PSCI 0.2 to allow sleeping via hypervisor calls
	initParams.features[0] |= 1 << KVM_ARM_VCPU_EL1_32BIT;
	initParams.features[0] |= 1 << KVM_ARM_VCPU_PSCI_0_2;

	printf("Target: %d\n", initParams.target);
	for (int i = 0; i < 7; i++) {
		printf("Features %d: %08X\n", i, initParams.features[i]);
	}

	// Properly hot-reset the CPU and initialize it
	if (ioctl(fd, KVM_ARM_VCPU_INIT, initParams) < 0) {
		printf("KVM_ARM_VCPU_INIT failed\n");
		return false;
	}

	// Get the register list available via KVM{S/G}_ET_ONE_REG. First we get the number of size to allocate with an ioctl that's going to fail.
	// Then we call the same ioctl again with the correct number of regs.
	kvm_reg_list tempRegList;
	tempRegList.n = 0;
	ioctl(fd, KVM_GET_REG_LIST, &tempRegList);

	// Properly allocate the new register list, copy the register number from tempRegList to regList
	regList = (kvm_reg_list*)malloc(sizeof(kvm_reg_list) + tempRegList.n * sizeof(u64));
	regList->n = tempRegList.n;
	if (ioctl(fd, KVM_GET_REG_LIST, regList) < 0) {
		printf("KVM_GET_REG_LIST failed\n");
		return false;
	}

	// Fetch indices for GPRs
	setupRegisterIndices();
	return true;
}

u32 KVM::VirtualCPU::getGPR(int index) {
	u64 ret;
	kvm_one_reg reg;

	reg.id = gprIDs[index];
	reg.addr = (u64)&ret;

	// If this fails everything will snap in half
	if (ioctl(fd, KVM_GET_ONE_REG, &reg) < 0) [[unlikely]] {
		printf("KVM::VirtualCPU::GetGPR failed\n");
	}

	return (u32)ret;
}

void KVM::VirtualCPU::setGPR(int index, u32 value) {
	u64 val = (u64)value;
	kvm_one_reg reg;

	reg.id = gprIDs[index];
	reg.addr = (u64)&val;

	// If this fails everything will also snap in half
	if (ioctl(fd, KVM_SET_ONE_REG, &reg) < 0) [[unlikely]] {
		printf("KVM::VirtualCPU::SetGPR failed\n");
	}
}

u32 KVM::VirtualCPU::getPstate() {
	u64 ret;
	kvm_one_reg reg;

	reg.id = AARCH64_CORE_REG(regs.pstate);
	reg.addr = (u64)&ret;

	if (ioctl(fd, KVM_GET_ONE_REG, &reg) < 0) [[unlikely]] {
		printf("KVM::VirtualCPU::GetPstate failed\n");
	}

	return (u32)ret;
}

void KVM::VirtualCPU::setPstate(u32 value) {
	u64 val = (u64)value;
	kvm_one_reg reg;

	reg.id = AARCH64_CORE_REG(regs.pstate);
	reg.addr = (u64)&val;

	if (ioctl(fd, KVM_SET_ONE_REG, &reg) < 0) [[unlikely]] {
		printf("KVM::VirtualCPU::SetPstate failed\n");
	}
}

u32 KVM::VirtualCPU::getPC() {
	u64 ret;
	kvm_one_reg reg;

	reg.id = AARCH64_CORE_REG(regs.pc);
	reg.addr = (u64)&ret;

	if (ioctl(fd, KVM_GET_ONE_REG, &reg) < 0) [[unlikely]] {
		printf("KVM::VirtualCPU::GetPC failed\n");
	}

	return (u32)ret;
}

void KVM::VirtualCPU::setPC(u32 value) {
	u64 val = (u64)value;
	kvm_one_reg reg;

	reg.id = AARCH64_CORE_REG(regs.pc);
	reg.addr = (u64)&val;

	if (ioctl(fd, KVM_SET_ONE_REG, &reg) < 0) [[unlikely]] {
		printf("KVM::VirtualCPU::SetPC failed\n");
	}
}

void KVM::VirtualCPU::run() {
	printf("Starting run (PC = %08X, pstate = %08X)\n", getPC(), getPstate());

	if (ioctl(fd, KVM_RUN, 0) < 0) [[unlikely]] {
		printf("KVM::VirtualCPU::Run failed\n");
	}

	printf("Done executing. Exit reason: %d, r6 = %08X\n", runInfo->exit_reason, getGPR(6));
}

void KVM::VirtualCPU::setupRegisterIndices() {
  // We can't use a loop here because the macro uses offsetof behind the scenes....
	gprIDs[0] = AARCH64_CORE_REG(regs.regs[0]);
	gprIDs[1] = AARCH64_CORE_REG(regs.regs[1]);
	gprIDs[2] = AARCH64_CORE_REG(regs.regs[2]);
	gprIDs[3] = AARCH64_CORE_REG(regs.regs[3]);
	gprIDs[4] = AARCH64_CORE_REG(regs.regs[4]);
	gprIDs[5] = AARCH64_CORE_REG(regs.regs[5]);
	gprIDs[6] = AARCH64_CORE_REG(regs.regs[6]);
	gprIDs[7] = AARCH64_CORE_REG(regs.regs[7]);
	gprIDs[8] = AARCH64_CORE_REG(regs.regs[8]);
	gprIDs[9] = AARCH64_CORE_REG(regs.regs[9]);
	gprIDs[10] = AARCH64_CORE_REG(regs.regs[10]);
	gprIDs[11] = AARCH64_CORE_REG(regs.regs[11]);
	gprIDs[12] = AARCH64_CORE_REG(regs.regs[12]);
	gprIDs[13] = AARCH64_CORE_REG(regs.regs[13]);
	gprIDs[14] = AARCH64_CORE_REG(regs.regs[14]);
	gprIDs[15] = AARCH64_CORE_REG(regs.regs[15]);
	// TODO: r16-31, implement syncing between the arm32 and arm64 register states
}

int main() {
	KVM::VirtualMachine vm;
	KVM::VirtualCPU vCPU;

	if (!vCPU.init(vm)) {
		printf("Failed to initialize virtual CPU\n");
		return -1;
	} else {
		printf("Initialized virtual CPU successfully\n");
	}

	vCPU.setPC(LOAD_ADDR);
	vCPU.run();

	return 0;
}

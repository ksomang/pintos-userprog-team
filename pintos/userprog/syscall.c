#include "userprog/syscall.h"
#include <stdio.h>
#include <syscall-nr.h>
#include "threads/interrupt.h"
#include "threads/thread.h"
#include "threads/loader.h"
#include "userprog/gdt.h"
#include "threads/flags.h"
#include "threads/init.h"
#include "intrinsic.h"

void syscall_entry (void);
void syscall_handler (struct intr_frame *);
static bool validate_buffer (char *buffer, size_t size);

/* System call.
 *
 * Previously system call services was handled by the interrupt handler
 * (e.g. int 0x80 in linux). However, in x86-64, the manufacturer supplies
 * efficient path for requesting the system call, the `syscall` instruction.
 *
 * The syscall instruction works by reading the values from the the Model
 * Specific Register (MSR). For the details, see the manual. */

#define MSR_STAR 0xc0000081         /* Segment selector msr */
#define MSR_LSTAR 0xc0000082        /* Long mode SYSCALL target */
#define MSR_SYSCALL_MASK 0xc0000084 /* Mask for the eflags */

void
syscall_init (void) {
	write_msr(MSR_STAR, ((uint64_t)SEL_UCSEG - 0x10) << 48  |
			((uint64_t)SEL_KCSEG) << 32);
	write_msr(MSR_LSTAR, (uint64_t) syscall_entry);

	/* The interrupt service rountine should not serve any interrupts
	 * until the syscall_entry swaps the userland stack to the kernel
	 * mode stack. Therefore, we masked the FLAG_FL. */
	write_msr(MSR_SYSCALL_MASK,
			FLAG_IF | FLAG_TF | FLAG_DF | FLAG_IOPL | FLAG_AC | FLAG_NT);
}

/* The main system call interface */
void
syscall_handler (struct intr_frame *f) {
	switch (f->R.rax) {
		case SYS_HALT:
			power_off ();
			break;
		case SYS_EXIT:
			thread_current ()->exit_status = (int) f->R.rdi;
			thread_exit ();
			break;
		case SYS_WRITE:
			if ((int) f->R.rdi == 1) {
				const char *buffer = (const char *) f->R.rsi;
				size_t size = (size_t) f->R.rdx;

				// 버퍼 주소 유효성 검사
				if (!validate_buffer (buffer, size)) {
					thread_exit ();
				}

				putbuf (buffer, size);
				f->R.rax = size;
			} else {
				f->R.rax = -1;
			}
			break;
		default:
			thread_exit ();
			break;
	}
}

// 사용자 프로그램이 NULL, 혹은 커널 주소로 접근하는 것을 방지
static bool
validate_buffer (char *buffer, size_t size) {
	if (buffer == NULL || !is_user_vaddr (buffer))
		return false;
	
	if (size == 0)
		return true;
	
	char *end = buffer + size - 1;

	// 끝점이 유저 영역인지 검사
	if (!is_user_vaddr (end))
		return false;
	
	// 페이지 매핑 검사
	for (char *addr = pg_round_down (buffer);
	addr <= end;
	addr += PGSIZE) {
		if (!pml4_get_page (thread_current ()->pml4, (const void *) addr))
			return false;
	}

	return true;
}
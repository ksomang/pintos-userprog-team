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
#include "threads/vaddr.h"
#include "threads/mmu.h"
#include "threads/synch.h"
#include "filesys/filesys.h"
#include "filesys/file.h"
#include "threads/palloc.h"
#include "devices/input.h"

void syscall_entry (void);
static bool check_address (const void *addr);
static bool check_buffer (const void *buffer, size_t size);
static bool check_string (const char *str);
static void sys_halt (void);
static int sys_write (int fd, const void *buffer, unsigned length);
static bool sys_create (const char *file, unsigned initial_size);
static bool sys_remove (const char *file);
static void sys_exit (int status);
static int sys_open (const char *file);
static void sys_close (int fd);
static int sys_read (int fd, void *buffer, unsigned size);
static struct file *fd_to_file (int fd);
void syscall_handler (struct intr_frame *);

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

/* 파일 시스템은 동기화가 안 되어 있으므로, 
파일 시스템 함수 호출을 감싸는 전역 lock. */
static struct lock filesys_lock;

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

	lock_init (&filesys_lock);
}

/* 현재 프로세스의 열린 파일을 모두 닫고, fd 표 페이지를 돌려준다.
   fd 표는 사용자 프로그램만 process_init에서 받는다.
   fd 표를 받은 적 없는 스레드면(fd_table이 NULL) 아무것도 하지 않는다. */
void 
destroy_fd_table (void) {
	struct thread *t = thread_current ();
	if (t->fd_table == NULL) {
		return;
	}

	lock_acquire (&filesys_lock);

	for (int fd = 2; fd < FD_MAX; fd++) {
		if (t->fd_table[fd] != NULL) {
			file_close (t->fd_table[fd]);
		}
	}

	lock_release (&filesys_lock);

	palloc_free_page (t->fd_table);
	t->fd_table = NULL;
}

/* 사용자가 넘긴 주소가 안전한지 검사한다.
   NULL, 커널 주소, 매핑되지 않은 주소면 false를 돌려준다.
   안전하면 true를 돌려준다.
   TODO: Project 3에서 lazy loading이 들어가면 수정이 필요하다. */
static bool 
check_address (const void *addr) {
	if (addr == NULL || !is_user_vaddr (addr) || pml4_get_page(thread_current ()->pml4, addr) == NULL) {
		return false;
	}
	return true;
}

/* BUFFER부터 SIZE바이트가 걸친 모든 페이지를 페이지마다 한 번씩 검사한다. */
static bool 
check_buffer (const void *buffer, size_t size) {
	if (size == 0)
		return true;

	uintptr_t start_page = (uintptr_t) pg_round_down (buffer);
	uintptr_t end = (uintptr_t) buffer + size - 1;

	for (uintptr_t page = start_page; page <= end; page += PGSIZE) {
		if (!check_address ((const void *) page)) {
			return false;
		}
	}
	return true;
}

/* 이름이 FILE인 파일을 열고, 새 fd를 돌려준다.
   파일이 없거나 fd 표에 빈 칸이 없으면 -1을 돌려준다.
   이름이 잘못된 주소면 프로세스를 exit(-1)로 끝낸다. */
static int 
sys_open (const char *file) {
	/* 파일 이름이 안전한 주소인지 lock 잡기 전에 검사한다. */
	if (!check_string (file)) {
		sys_exit (-1);
	}

	/* fd 표를 받지 못한 프로세스는 파일을 열 수 없다. */
	struct file **fd_table = thread_current ()->fd_table;

	if (fd_table == NULL) {
		return -1;
	}

	/* 2번부터 가장 작은 빈 칸을 찾는다. (0, 1은 콘솔용)
	   빈 칸을 먼저 찾아서, 칸이 없을 때 파일을 괜히 열지 않는다. */
	int fd;

	for (fd = 2; fd < FD_MAX; fd++) {
		if (fd_table[fd] == NULL) {
			break;
		}
	}

	/* 끝까지 돌았는데 빈 칸이 없다. */
	if (fd == FD_MAX) {
		return -1;
	}

	/* 파일 시스템은 동기화가 안 되어 있어서 lock 안에서 연다. */
	lock_acquire (&filesys_lock);
	struct file *opened = filesys_open (file);
	lock_release (&filesys_lock);

	/* 없는 파일이거나 이름이 ""이면 열리지 않는다. */
	if (opened == NULL) {
		return -1;
	}

	/* 찾은 빈 칸에 열린 파일을 넣고, 그 칸 번호를 fd로 돌려준다. */
	fd_table[fd] = opened;

	return fd;
}

/* FD에 해당하는 열린 파일을 닫는다.
   범위를 벗어난 FD일 경우 프로세스를 종료시킨다. */
static void
sys_close (int fd) {
	if (!fd_to_file (fd))
		sys_exit (-1);
	
	struct thread *t = thread_current ();
	
	lock_acquire (&filesys_lock);
	
	file_close (t->fd_table[fd]);
	t->fd_table[fd] = NULL;
	
	lock_release (&filesys_lock);
}

static int 
sys_read (int fd, void *buffer, unsigned size) {
	if (!check_buffer (buffer, size)) {
		sys_exit (-1);
	}

	if (fd == 0) {
		char *buf = (char *) buffer;

		for (unsigned i = 0; i < size; i++) {
			buf[i] = input_getc ();
		}
		return size;
	}

	struct file *file = fd_to_file (fd);

	if (file == NULL) {
		sys_exit (-1);
	}

	lock_acquire (&filesys_lock);

	int bytes_read = file_read (file, buffer, size);

	lock_release (&filesys_lock);

	return bytes_read;
}

/* FD에 해당하는 열린 파일을 돌려준다.
   FD가 범위 밖(2 미만, FD_MAX 이상)이거나 빈 칸이면 NULL을 돌려준다.
   fd 0, 1(콘솔)은 여기서 다루지 않는다. */
static struct file *
fd_to_file (int fd) {
	struct file **fd_table = thread_current ()->fd_table;

	if (fd_table == NULL || fd < 2 || fd >= FD_MAX) {
		return NULL;
	}

	return fd_table[fd];
}

/* 문자열 STR을 '\0'까지 따라가며, 첫 글자와 새 페이지에 들어설 때마다 검사한다. */
static bool 
check_string (const char *str) {
	const char *p = str;

	while (true) {
		if (p == str || pg_ofs (p) == 0) {
			if (!check_address (p)) {
				return false;
			}
		}

		if (*p == '\0') {
			return true;
		}

		p++;
	}
	return true;
}

/* Pintos를 종료한다. */
static void 
sys_halt (void) {
	power_off ();
}

/* FD에 BUFFER의 LENGTH바이트를 쓰고, 실제로 쓴 바이트 수를 돌려준다.
   버퍼가 잘못된 주소면 프로세스를 exit(-1)로 끝낸다. */
static int 
sys_write (int fd, const void *buffer, unsigned length) {
	if (!check_buffer (buffer, length)) {
		sys_exit (-1);
	}

	if (fd == 1) {
		putbuf (buffer, length);
		return length;
	}

	struct file *file = fd_to_file (fd);

	if (file == NULL) {
		sys_exit (-1);
	}

	lock_acquire (&filesys_lock);

	int bytes_write = file_write (file, buffer, length);

	lock_release (&filesys_lock);

	return bytes_write;
}

/* 이름이 FILE이고 크기가 INITIAL_SIZE인 파일을 만든다. 성공하면 true.
이름이 잘못된 주소면 프로세스를 exit(-1)로 끝낸다. */
static bool 
sys_create (const char *file, unsigned initial_size) {
	/* lock 잡기 전에 파일 이름 검사 */
	if (!check_string (file)) {
		sys_exit (-1);
	}

	/* lock 잡기 */
	lock_acquire (&filesys_lock);

	/* 파일 만들고 결과를 변수에 저장 */
	bool result = filesys_create (file, initial_size);

	/* lock 풀기 */
	lock_release (&filesys_lock);

	return result;
}

/* 이름이 FILE인 파일을 지운다. 성공하면 true, 실패하면 false.
   이름이 잘못된 주소면 프로세스를 exit(-1)로 끝낸다. */
static bool 
sys_remove (const char *file) {
	/* lock 잡기 전에 파일 이름 검사 */
	if (!check_string (file)) {
		sys_exit (-1);
	}

	/* lock 잡기 */
	lock_acquire (&filesys_lock);

	/* 파일 지우고 결과를 변수에 저장 */
	bool result = filesys_remove (file);

	/* lock 풀기 */
	lock_release (&filesys_lock);

	return result;
}

/* 현재 프로세스를 종료 코드 STATUS로 끝낸다. */
static void 
sys_exit (int status) {
	thread_current ()->exit_status = status;
	thread_exit ();
}

/* The main system call interface */
void
syscall_handler (struct intr_frame *f) {
	switch (f->R.rax) {
		case SYS_HALT:
			sys_halt ();
			break;
		case SYS_EXIT:
			sys_exit ((int) f->R.rdi);
			break;
		case SYS_READ:
			f->R.rax = sys_read ((int) f->R.rdi, 
								(void *) f->R.rsi, 
								(unsigned) f->R.rdx);
			break;
		case SYS_WRITE:
			f->R.rax = sys_write ((int) f->R.rdi, 
								  (const void *) f->R.rsi,
								  (unsigned) f->R.rdx);
			break;
		case SYS_CREATE:
			f->R.rax = sys_create ((const char *) f->R.rdi, 
								   (unsigned) f->R.rsi);
			break;
		case SYS_REMOVE:
			f->R.rax = sys_remove ((const char *) f->R.rdi);
			break;
		case SYS_OPEN:
			f->R.rax = sys_open ((const char *) f->R.rdi);
			break;
		case SYS_CLOSE:
			sys_close ((int) f->R.rdi);
			break;
		default:
			sys_exit (-1);
			break;
	}
}
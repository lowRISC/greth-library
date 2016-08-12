/* Copyright (c) 2016 by the author(s)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * =============================================================================
 *
 * Author(s):
 *   Stefan Wallentowitz <stefan.wallentowitz@tum.de>
 *   Wei Song <wsong83@gmail.com>
 */

#include <string.h>
#include "GlipTcp.h"

enum {STATE_MASK_CTRL  = 1,
      STATE_MASK_READ  = 2,
      STATE_MASK_WRITE = 4,
      UART_DELAY = 8};

enum {SIZ = 4,
      WADDR = 0x80000000,
      RADDR = 0x80000004,
      SADDR = 0x80000008};

void glip_tcp_toplevel(void *obj)
{
  uint64_t status;
  static char tmp[100], old[100];
  int connected = glip_tcp_connected(obj);
  if (connected <= 0) {
    long data = (1<<16);
    edcl_write(WADDR, SIZ, (uint8_t *)&data);
  }
  else
    {
      int state = glip_tcp_next_cycle(obj);
      edcl_read(SADDR, SIZ, (uint8_t *)&status);
      int owrcnt = (status >> 16) & 7;
      int iwrcnt = (status >> 19) & 7;
      int ordcnt = (status >> 22) & 7;
      int irdcnt = (status >> 25) & 7;
      int oempty = (status >> 28) & 1;
      int iempty = (status >> 29) & 1;
      int oerror = (status >> 30) & 1;
      int ierror = (status >> 31) & 1;
#ifdef VERBOSE
      sprintf(tmp, "state = %X, iempty = %d, oempty = %d, iwrcnt=%d, owrcnt=%d", state, iempty, oempty, iwrcnt, owrcnt);
      if (strcmp(old, tmp))
	{
	  puts(tmp);
	  strcpy(old, tmp);
	}
#endif
      // Get control message
      if ((state & STATE_MASK_CTRL) != 0) {
	long data = glip_tcp_control_msg(obj) & 1 ? 1<<17 : 0;
	edcl_write(WADDR, SIZ, (uint8_t *)&data);
      }
      
      // We have new incoming data
      if ((state & STATE_MASK_READ) != 0) {
	long data = glip_tcp_read(obj) & (1<<WIDTH)-1;
	edcl_write(WADDR, SIZ, (uint8_t *)&data);
	glip_tcp_read_ack(obj);
      }
      
      // Write outgoing data
      if (((state & STATE_MASK_WRITE) != 0) && !oempty) {
	long data = 0;
	edcl_read(RADDR, SIZ, (uint8_t *)&data);
	glip_tcp_write(obj, data & (1<<WIDTH)-1);
      }
    }
}

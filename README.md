![CI Status](https://github.com/libcsp/libcsp/actions/workflows/build-test.yml/badge.svg)

# The Cubesat Space Protocol

![CSP](./doc/_images/csp.png)

Cubesat Space Protocol (CSP) is a small protocol stack written in C. CSP
is designed to ease communication between distributed embedded systems
in smaller networks, such as Cubesats. The design follows the TCP/IP
model and includes a transport protocol, a routing protocol and several
MAC-layer interfaces. The core of `libcsp`
includes a router, a connection oriented socket API and
message/connection pools.

The protocol is based on an very lightweight header containing both transport and
network-layer information. Its implementation is designed for, but not
limited to, embedded systems with very limited CPU and memory resources.
The implementation is written in GNU C and is currently ported to run on FreeRTOS, Zephyr
and Linux (POSIX).

The idea is to give sub-system developers of cubesats the same features
of a TCP/IP stack, but without adding the huge overhead of the IP
header. The small footprint and simple implementation allows a small
8-bit system to be fully connected on the network. This allows all
subsystems to provide their services on the same network level, without
any master node required. Using a service oriented architecture has
several advantages compared to the traditional master/slave topology used
on many cubesats.

  - Standardised network protocol: All subsystems can communicate with
    each other (multi-master)
  - Service loose coupling: Services maintain a relationship that
    minimizes dependencies between subsystems
  - Service abstraction: Beyond descriptions in the service contract,
    services hide logic from the outside world
  - Service reusability: Logic is divided into services with the
    intention of promoting reuse.
  - Service autonomy: Services have control over the logic they
    encapsulate.
  - Service Redundancy: Easily add redundant services to the bus
  - Reduces single point of failure: The complexity is moved from a
    single master node to several well defined services on the network

The implementation of `libcsp` is written
with simplicity in mind, but it's compile time configuration allows it
to have some rather advanced features as well.

## Features

  - Thread safe Socket API
  - Router task with Quality of Services
  - Connection-oriented operation (RFC 908 and 1151).
  - Connection-less operation (similar to UDP)
  - ICMP-like requests such as ping and buffer status.
  - Loopback interface
  - Very Small Footprint in regards to code and memory required
  - Zero-copy buffer and queue system
  - Modular network interface system
  - OS abstraction, currently ported to: FreeRTOS, Zephyr, Linux
  - Broadcast traffic
  - Promiscuous mode

## Documentation

The latest version of the /doc folder is compiled to HTML and hosted on:

  [libcsp.github.io/libcsp/](https://libcsp.github.io/libcsp/)

## Zephyr interface configuration and lifecycle

Applications select hardware devices and supply interface names, node addresses,
CAN bitrates/filters, and UART parameters. The driver APIs do not read application
settings or choose a topology. Enable the required devices and driver support in
the application's devicetree and Kconfig.

The Zephyr adapters support deferred reception so the CSP core, interface policy,
and routing table can be ready before incoming packets are processed:

1. Set `csp_conf.version` and call `csp_init()` before preparing interfaces.
2. Prepare/register each interface using the APIs below.
3. Set its `netmask` (prefix length) and `is_default` policy, then install routes.
4. Start reception and run the application's CSP router/server tasks.
5. On failure, clean up acquired interfaces in reverse order and check cleanup
   return values before attempting another initialization.

| Transport | Prepare without enabling RX | Enable RX | Cleanup |
|---|---|---|---|
| CAN | `csp_can_prepare_interface()` | `csp_can_start()` | `csp_can_stop()` |
| UART KISS | `csp_kiss_prepare()` | `csp_kiss_start()` | `csp_kiss_close()` |
| Raw UART | `csp_usart_prepare()` | `csp_usart_start()` | `csp_usart_close()` |

CAN declarations are in [`can_zephyr.h`](include/csp/drivers/can_zephyr.h);
UART/KISS declarations are in
[`usart_zephyr.h`](include/csp/drivers/usart_zephyr.h).
Raw UART preparation reserves the UART and callback resources but does not
register a CSP interface. KISS preparation also registers the CSP interface.
Only one CSP owner can use a UART device at a time.

UART preparation takes a complete Zephyr `struct uart_config`: baud rate, data
bits, stop bits, parity, and flow control. It applies this configuration before
enabling reception, rather than relying on devicetree `current-speed`.
Enable `CONFIG_UART_USE_RUNTIME_CONFIGURE`; the driver must support
`uart_config_get()` and `uart_configure()`. UART cleanup restores the previous
configuration. CAN cleanup stops the controller/RX thread and releases the
registered interface and filter resources.

Provide enough RX slots for the requested topology using
`CONFIG_CSP_CAN_RX_THREAD_NUM` and `CONFIG_CSP_UART_RX_THREAD_NUM`.

### Existing driver APIs

`csp_can_open_and_add_interface()`,
`csp_usart_open_and_add_kiss_interface()`, and `csp_usart_open()` remain available
and retain immediate-start behavior. The deferred APIs are intended for callers
that need to install policy and routes before reception starts.

Applications using the separate csp-service module can delegate this lifecycle
to `csp_service_init_config()` and call `csp_service_start()` after success;
these are service-layer APIs, not libcsp APIs.

## Routing table capacity and string limits

The routing table has two independent limits:

| Limit | Generated C macro | Zephyr Kconfig option | Default |
|---|---|---|---|
| Number of route entries | `CSP_RTABLE_SIZE` | `CONFIG_CSP_RTABLE_SIZE` | 10 |
| Routing string characters, excluding the terminating null byte | `CSP_RTABLE_STRING_MAX_LEN` | `CONFIG_CSP_RTABLE_STRING_MAX_LEN` | 100 |

For example, `"16/5 CAN1, 8/5 KISS, 0/0 CAN0"` contains three route entries.
Interface names must match registered interfaces. `csp_rtable_check_config()`
can validate text against a proposed protocol version and interface-name list
before those interfaces are registered; `csp_rtable_check()` validates against
the current CSP interfaces, and `csp_rtable_load()` validates and installs routes.
See [`csp_rtable.h`](include/csp/csp_rtable.h).

Configure the limits in the application's `prj.conf`:

```conf
CONFIG_CSP_USE_RTABLE=y
CONFIG_CSP_RTABLE_SIZE=10
CONFIG_CSP_RTABLE_STRING_MAX_LEN=256
```

The string parser rejects text exceeding the configured character limit and
uses a stack buffer of `CSP_RTABLE_STRING_MAX_LEN + 1` bytes. Zephyr permits
limits from 1 to 1024; account for the buffer when sizing the calling thread's
stack. Applications can size saved route buffers with
`CONFIG_CSP_RTABLE_STRING_MAX_LEN + 1` to share the same limit.

Other build systems expose equivalent settings:

| Build system | Entry count | String character limit |
|---|---|---|
| CMake | `-DCSP_RTABLE_SIZE=10` | `-DCSP_RTABLE_STRING_MAX_LEN=256` |
| Meson | `-Drtable_size=10` | `-Drtable_string_max_len=256` |
| Waf | `--with-rtable-size=10` | `--with-rtable-string-max-len=256` |

Reconfigure/rebuild after changing these options; do not edit generated
`csp/autoconfig.h` directly.

## Contributing

Thank you for considering contributing to libcsp! We welcome
contributions from the community to help improve and grow the
project. Please take a moment to review our
[guidelines](./doc/contributing.md) before opening a Pull Request!

## Software license

The source code is available under MIT license, see LICENSE for license text

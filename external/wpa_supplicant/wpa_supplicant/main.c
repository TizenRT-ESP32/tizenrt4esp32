/*
 * WPA Supplicant / main() function for UNIX like OSes and MinGW
 * Copyright (c) 2003-2013, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#include "includes.h"
#include <fcntl.h>

#include "common.h"
#include "wpa_supplicant_i.h"
#include "driver_i.h"
#include "p2p_supplicant.h"

static void usage(void)
{
	int i;
	printf("%s\n\n%s\n" "usage:\n" "  wpa_supplicant [-BddhKLqq"
#ifdef CONFIG_DEBUG_SYSLOG
		   "s"
#endif							/* CONFIG_DEBUG_SYSLOG */
		   "t"
#ifdef CONFIG_DBUS
		   "u"
#endif							/* CONFIG_DBUS */
		   "vW] [-P<pid file>] " "[-g<global ctrl>] \\\n" "        [-G<group>] \\\n" "        -i<ifname> -c<config file> [-C<ctrl>] [-D<driver>] " "[-p<driver_param>] \\\n" "        [-b<br_ifname>] [-e<entropy file>]"
#ifdef CONFIG_DEBUG_FILE
		   " [-f<debug file>]"
#endif							/* CONFIG_DEBUG_FILE */
		   " \\\n" "        [-o<override driver>] [-O<override ctrl>] \\\n" "        [-N -i<ifname> -c<conf> [-C<ctrl>] " "[-D<driver>] \\\n"
#ifdef CONFIG_P2P
		   "        [-m<P2P Device config file>] \\\n"
#endif							/* CONFIG_P2P */
		   "        [-p<driver_param>] [-b<br_ifname>] [-I<config file>] " "...]\n" "\n" "drivers:\n", wpa_supplicant_version, wpa_supplicant_license);

	for (i = 0; wpa_drivers[i]; i++) {
		printf("  %s = %s\n", wpa_drivers[i]->name, wpa_drivers[i]->desc);
	}

#ifndef CONFIG_NO_STDOUT_DEBUG
	printf("options:\n" "  -b = optional bridge interface name\n" "  -B = run daemon in the background\n" "  -c = Configuration file\n" "  -C = ctrl_interface parameter (only used if -c is not)\n" "  -i = interface name\n" "  -I = additional configuration file\n" "  -d = increase debugging verbosity (-dd even more)\n" "  -D = driver name (can be multiple drivers: nl80211,wext)\n" "  -e = entropy file\n");
#ifdef CONFIG_DEBUG_FILE
	printf("  -f = log output to debug file instead of stdout\n");
#endif							/* CONFIG_DEBUG_FILE */
	printf("  -g = global ctrl_interface\n" "  -G = global ctrl_interface group\n" "  -K = include keys (passwords, etc.) in debug output\n");
#ifdef CONFIG_DEBUG_SYSLOG
	printf("  -s = log output to syslog instead of stdout\n");
#endif							/* CONFIG_DEBUG_SYSLOG */
#ifdef CONFIG_DEBUG_LINUX_TRACING
	printf("  -T = record to Linux tracing in addition to logging\n");
	printf("       (records all messages regardless of debug verbosity)\n");
#endif							/* CONFIG_DEBUG_LINUX_TRACING */
	printf("  -t = include timestamp in debug messages\n" "  -h = show this help text\n" "  -L = show license (BSD)\n" "  -o = override driver parameter for new interfaces\n" "  -O = override ctrl_interface parameter for new interfaces\n" "  -p = driver parameters\n" "  -P = PID file\n" "  -q = decrease debugging verbosity (-qq even less)\n");
#ifdef CONFIG_DBUS
	printf("  -u = enable DBus control interface\n");
#endif							/* CONFIG_DBUS */
	printf("  -v = show version\n" "  -W = wait for a control interface monitor before starting\n"
#ifdef CONFIG_P2P
		   "  -m = Configuration file for the P2P Device interface\n"
#endif							/* CONFIG_P2P */
		   "  -N = start describing new interface\n");

	printf("example:\n" "  wpa_supplicant -D%s -iwl1 -c/mnt/wifi/wpa_supplicant.conf\n", wpa_drivers[0] ? wpa_drivers[0]->name : "nl80211");
#endif							/* CONFIG_NO_STDOUT_DEBUG */
}

static void license(void)
{
#ifndef CONFIG_NO_STDOUT_DEBUG
	printf("%s\n\n%s%s%s%s%s\n", wpa_supplicant_version, wpa_supplicant_full_license1, wpa_supplicant_full_license2, wpa_supplicant_full_license3, wpa_supplicant_full_license4, wpa_supplicant_full_license5);
#endif							/* CONFIG_NO_STDOUT_DEBUG */
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int wpa_supplicant_main(int argc, char *argv[])
#endif
{
	int c, i, nullfd;
	struct wpa_interface *ifaces, *iface;
	int iface_count, exitcode = -1;
	struct wpa_params params;
	struct wpa_global *global;

	if (os_program_init()) {
		return -1;
	}

	os_memset(&params, 0, sizeof(params));

#ifdef CONFIG_DEBUG_WLAN_SUPPLICANT_ERROR
	params.wpa_debug_level = MSG_ERROR;
#else
	params.wpa_debug_level = MSG_INFO;
#endif

	iface = ifaces = os_zalloc(sizeof(struct wpa_interface));
	if (ifaces == NULL) {
		return -1;
	}
	iface_count = 1;

	optind = 0;
	for (;;) {
		c = getopt(argc, argv, "b:Bc:C:D:de:f:g:G:hi:I:KLm:No:O:p:P:qsTtuvW");
		if (c < 0) {
			break;
		}
		switch (c) {
		case 'b':
			iface->bridge_ifname = optarg;
			break;
		case 'B':
			params.daemonize++;
			break;
		case 'c':
			iface->confname = optarg;
			break;
		case 'C':
			iface->ctrl_interface = optarg;
			break;
		case 'D':
			iface->driver = optarg;
			break;
		case 'd':
#ifdef CONFIG_NO_STDOUT_DEBUG
			printf("Debugging disabled with " "CONFIG_NO_STDOUT_DEBUG=y build time " "option.\n");
			goto out;
#else							/* CONFIG_NO_STDOUT_DEBUG */
			params.wpa_debug_level--;
			break;
#endif							/* CONFIG_NO_STDOUT_DEBUG */
		case 'e':
			params.entropy_file = optarg;
			break;
#ifdef CONFIG_DEBUG_FILE
		case 'f':
			params.wpa_debug_file_path = optarg;
			break;
#endif							/* CONFIG_DEBUG_FILE */
		case 'g':
			params.ctrl_interface = optarg;
			break;
		case 'G':
			params.ctrl_interface_group = optarg;
			break;
		case 'h':
			usage();
			exitcode = 0;
			goto out;
		case 'i':
			iface->ifname = optarg;
			break;
		case 'I':
			iface->confanother = optarg;
			break;
		case 'K':
			params.wpa_debug_show_keys++;
			break;
		case 'L':
			license();
			exitcode = 0;
			goto out;
#ifdef CONFIG_P2P
		case 'm':
			params.conf_p2p_dev = optarg;
			break;
#endif							/* CONFIG_P2P */
		case 'o':
			params.override_driver = optarg;
			break;
		case 'O':
			params.override_ctrl_interface = optarg;
			break;
		case 'p':
			iface->driver_param = optarg;
			break;
		case 'P':
			os_free(params.pid_file);
			params.pid_file = os_rel2abs_path(optarg);
			break;
		case 'q':
			params.wpa_debug_level++;
			break;
#ifdef CONFIG_DEBUG_SYSLOG
		case 's':
			params.wpa_debug_syslog++;
			break;
#endif							/* CONFIG_DEBUG_SYSLOG */
#ifdef CONFIG_DEBUG_LINUX_TRACING
		case 'T':
			params.wpa_debug_tracing++;
			break;
#endif							/* CONFIG_DEBUG_LINUX_TRACING */
		case 't':
			params.wpa_debug_timestamp++;
			break;
#ifdef CONFIG_DBUS
		case 'u':
			params.dbus_ctrl_interface = 1;
			break;
#endif							/* CONFIG_DBUS */
		case 'v':
			printf("%s\n", wpa_supplicant_version);
			exitcode = 0;
			goto out;
		case 'W':
			params.wait_for_monitor++;
			break;
		case 'N':
			iface_count++;
			iface = os_realloc_array(ifaces, iface_count, sizeof(struct wpa_interface));
			if (iface == NULL) {
				goto out;
			}
			ifaces = iface;
			iface = &ifaces[iface_count - 1];
			os_memset(iface, 0, sizeof(*iface));
			break;
		default:
			usage();
			exitcode = 0;
			goto out;
		}
	}

	exitcode = 0;

	if (params.daemonize) {
		nullfd = open("/dev/null", O_RDWR);
		if (nullfd < 0) {
			printf("ERROR: Failed to open /dev/null for redirection\n");
		} else {
			printf("Starting supplicant as daemon...\n");

			close(0);
			close(1);
			close(2);
			(void)dup2(nullfd, 0);
			(void)dup2(nullfd, 1);
			(void)dup2(nullfd, 2);
		}
		printf("This text should not be visible when daemon!!!\n");
		params.daemonize--;
	} else {
		printf("Starting supplicant in foreground...\n");
	}

	global = wpa_supplicant_init(&params);
	if (global == NULL) {
		wpa_printf(MSG_ERROR, "Failed to initialize wpa_supplicant");
		exitcode = -1;
		goto out;
	} else {
		wpa_printf(MSG_INFO, "Successfully initialized wpa_supplicant");
	}

	for (i = 0; exitcode == 0 && i < iface_count; i++) {
		struct wpa_supplicant *wpa_s;

		if ((ifaces[i].confname == NULL && ifaces[i].ctrl_interface == NULL) || ifaces[i].ifname == NULL) {
			if (iface_count == 1 && (params.ctrl_interface || params.dbus_ctrl_interface)) {
				break;
			}
			usage();
			exitcode = -1;
			break;
		}
		wpa_s = wpa_supplicant_add_iface(global, &ifaces[i], NULL);
		if (wpa_s == NULL) {
			exitcode = -1;
			break;
		}
	}

	if (exitcode == 0) {
		exitcode = wpa_supplicant_run(global);
	}

	wpa_supplicant_deinit(global);

out:
	os_free(ifaces);
	os_free(params.pid_file);

	os_program_deinit();

	return exitcode;
}

int wpa_supplicant_task(int argc, char *argv[])
{
	pid_t task;

	argv[argc] = NULL;

	task = task_create("WPA Supplicant", CONFIG_WPA_SUPPLICANT_PRIORITY, CONFIG_WPA_SUPPLICANT_STACKSIZE, wpa_supplicant_main,	/*CONFIG_WPA_SUPPLICANT_ENTRYPOINT, */
					   &argv[1]);

	sleep(1);

	if (task < 0) {
		printf("ERROR: Failed wpa_supplicant_task\n");
		return -1;;
	}

	return 0;
}

/****************************************************************************
 *
 * Copyright 2017 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/

#define _POSIX_C_SOURCE 200809L
#define _BSD_SOURCE				// for the usleep
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>

#include "ocpayload.h"

#include "things_def.h"
#include "things_common.h"
#include "logging/things_logger.h"
#include "utils/things_malloc.h"
#include "cacommon.h"
#include "utils/things_string.h"
#include "things_resource.h"
#include "utils/things_network.h"
#include "things_api.h"
#include "things_types.h"
#include "things_server_builder.h"
#include "things_data_manager.h"

#include "things_req_handler.h"
#include "utils/things_hashmap.h"

#ifdef __ST_THINGS_RTOS__
#include "ctype.h"
#include "utils/things_rtos_util.h"
#endif

#define TAG "[things_reqhdlr]"

static handle_request_func_type g_handle_request_get_cb = NULL;
static handle_request_func_type g_handle_request_set_cb = NULL;

extern things_server_builder_s *g_builder;

static int g_handle_res_cnt = 0;
static char **g_handle_res_list = NULL;
static handle_request_func_type g_handle_req_cb = NULL;
static get_notify_obs_uri_cb g_get_notify_obs_uri = NULL;

static handle_request_interface_cb g_handle_request_interface_cb = NULL;

static stop_softap_func_type g_stop_soft_ap_cb = NULL;

static pthread_t g_req_handle;

static int g_quit_flag = 0;

OCEntityHandlerResult things_abort(pthread_t *h_thread_abort, things_es_enrollee_abort_e level);

/**
 * Return : 0 (Invalid Request), 1 (Resource Supporting Interface)
 * Need refactoring later...
 */
static int verify_request(OCEntityHandlerRequest *eh_request, const char *uri, int req_type)
{
	int result = 0;
	things_resource_s *pst_temp_resource = NULL;

	if (g_builder == NULL) {
		THINGS_LOG_E(TAG, "Server Builder is not registered..");
		goto EXIT_VALIDATION;
	}

	things_resource_s *resource = things_create_resource_inst(eh_request->requestHandle,
								  eh_request->resource,
								  eh_request->query,
								  eh_request->payload);	//g_builder->get_resource(g_builder, uri);
	things_resource_s *child = NULL;
	/*! Added by st_things for memory Leak fix
	 */
	pst_temp_resource = resource;
	if (resource == NULL) {
		THINGS_LOG_E(TAG, "Resource Not found : %s ", uri);
		goto EXIT_VALIDATION;
	}

	THINGS_LOG_D(TAG, "Query in the Request : %s", eh_request->query);

	//  Verify received request in valid or not..
	// 1. If there's no interface type in the request ..
	if ((strstr(eh_request->query, "if=") == NULL)
		|| (strstr(eh_request->query, OIC_INTERFACE_BASELINE) != NULL)
		|| (true == resource->things_is_supporting_interface_type(resource, eh_request->query))) {
		result = 1;
	}
	// 2. If request type == POST(OC_REST_POST == 4),
	//    then validate the attributes kyes in the request
	if (result == 1 && req_type == OC_REST_POST) {
		//  Reseting the result value to apply
		// the result of additional verification
		result = 0;

		//  If the given resource does not have sensor nor read interface type..
		if ((resource->things_is_supporting_interface_type(resource, OIC_INTERFACE_ACTUATOR))
			|| (!(resource->things_is_supporting_interface_type(resource, OIC_INTERFACE_SENSOR))
			&& !(resource->things_is_supporting_interface_type(resource, OC_RSRVD_INTERFACE_READ)))) {
			//  If no query or..
			//       query with "supporting interface types"  then...
			if ((strstr(eh_request->query, "if=") == NULL)
				|| ((strstr(eh_request->query, "if=") != NULL)
					&& (resource->things_is_supporting_interface_type(resource, eh_request->query))
				   )
			   ) {
				//  Verify request(Attributes.) with Validator func implemented
				const int num = resource->things_get_num_of_res_types(resource);

				THINGS_LOG_D(TAG, "# of RT :%d", num);

				if (num > 0) {
					for (int iter = 0; iter < num; iter++) {
						// if it's okey in any case then....it's valid..
						result |= dm_validate_attribute_in_request(resource->things_get_res_type(resource, iter), (const void *)eh_request->payload);
					}
				}
			}
		}
	}

EXIT_VALIDATION:

	/*! Added by st_things for memory Leak fix
	 */
	things_release_resource_inst(pst_temp_resource);

	THINGS_LOG_D(TAG, "Validation Result.(%d)", result);

	return result;
}

OCEntityHandlerResult send_response(OCRequestHandle request_handle, OCResourceHandle resource, OCEntityHandlerResult result, const char *uri, void *payload)
{
	THINGS_LOG_D(TAG, THINGS_FUNC_ENTRY);
	OCEntityHandlerResult eh_result = OC_EH_OK;
	OCEntityHandlerResponse response = { 0, 0, OC_EH_ERROR, 0, 0, {}, {0}, false };

	THINGS_LOG_D(TAG, "Creating Response with Request Handle : %x,Resource Handle : %x", request_handle, resource);

	response.numSendVendorSpecificHeaderOptions = 0;
	memset(response.sendVendorSpecificHeaderOptions, 0, sizeof response.sendVendorSpecificHeaderOptions);
	memset(response.resourceUri, 0, sizeof(response.resourceUri));

	if (NULL != request_handle && NULL != resource) {
		response.requestHandle = request_handle;	//eh_request->request_handle;
		response.resourceHandle = resource;	//eh_request->resource;
		response.persistentBufferFlag = 0;
		response.ehResult = result;
		if (payload != NULL) {
			THINGS_LOG_D(TAG, "Payload is Not NULL");
			response.payload = (OCPayload *) payload;
		} else {
			THINGS_LOG_D(TAG, "No Payload");
			response.payload = NULL;
		}

		THINGS_LOG_V(TAG, "\t\t\tRes. : %s ( %d )", (response.ehResult == OC_EH_OK ? "NORMAL" : "ERROR"), response.ehResult);

		OCStackResult ret = OCDoResponse(&response);
		THINGS_LOG_V(TAG, "\t\t\tMsg. Out? : (%s)", (ret == OC_STACK_OK) ? "SUCCESS" : "FAIL");
		if (ret != OC_STACK_OK) {
			eh_result = OC_EH_ERROR;
		}
	} else {
		eh_result = OC_EH_ERROR;
	}

	// THINGS_LOG_D(TAG, THINGS_FUNC_EXIT);
	return eh_result;
}

/**
 * This functions will replace the get_ap_scan_result(~~)
 */
static OCEntityHandlerResult get_provisioning_info(things_resource_s *target_resource)
{
	OCEntityHandlerResult eh_result = OC_EH_ERROR;

	const char *device_id = OCGetServerInstanceIDString();
	const char *device_rt = dm_get_things_device_type(0);
	bool is_owned = false;
	bool is_reset = things_get_reset_mask(RST_ALL_FLAG);

	if (OC_STACK_OK != OCGetDeviceOwnedState(&is_owned)) {
		THINGS_LOG_E(TAG, "Failed to get device owned state, Informing as UNOWNED~!!!!");
		is_owned = false;
	}

	if (target_resource->rep == NULL) {
		things_representation_s *pstRep = NULL;
		pstRep = things_create_representation_inst(NULL);
		target_resource->things_set_representation(target_resource, pstRep);
	}

	things_representation_s *child_rep[1] = { NULL };

	child_rep[0] = things_create_representation_inst(NULL);
	child_rep[0]->things_set_value(child_rep[0], SEC_ATTRIBUTE_PROV_TARGET_ID, device_id);
	child_rep[0]->things_set_value(child_rep[0], SEC_ATTRIBUTE_PROV_TARGET_RT, device_rt);
	child_rep[0]->things_set_bool_value(child_rep[0], SEC_ATTRIBUTE_PROV_TARGET_PUBED, dm_is_rsc_published());

	target_resource->rep->things_set_arrayvalue(target_resource->rep, SEC_ATTRIBUTE_PROV_TARGETS, 1, child_rep);
	target_resource->rep->things_set_bool_value(target_resource->rep, SEC_ATTRIBUTE_PROV_TARGET_OWNED, is_owned);
	target_resource->rep->things_set_value(target_resource->rep, SEC_ATTRIBUTE_PROV_EZSETDI, device_id);
	target_resource->rep->things_set_bool_value(target_resource->rep, SEC_ATTRIBUTE_PROV_RESET, is_reset);
	target_resource->rep->things_set_int_value(target_resource->rep, SEC_ATTRIBUTE_PROV_ABORT, 0);

	if (NULL != child_rep[0]) {
		things_release_representation_inst(child_rep[0]);
	}

	eh_result = OC_EH_OK;

	return eh_result;
}

static OCEntityHandlerResult trigger_reset_request(things_resource_s *target_resource, bool reset)
{
	OCEntityHandlerResult eh_result = OC_EH_ERROR;
	things_resource_s *clone_resource = NULL;
	bool isOwned;

	OCGetDeviceOwnedState(&isOwned);

	if (isOwned == false) {
		return OC_EH_NOT_ACCEPTABLE;
	}

	THINGS_LOG_D(TAG, "==> RESET : %s", (reset == true ? "YES" : "NO"));

	if (reset == true) {
		int res = -1;

		clone_resource = clone_resource_inst(target_resource);
		res = things_reset((void *)clone_resource, RST_NEED_CONFIRM);

		switch (res) {
		case 1:
			THINGS_LOG_D(TAG, "Reset Thread create is success.");
			eh_result = OC_EH_SLOW;
			break;
		case 0:
			THINGS_LOG_V(TAG, "Already Run Reset-Process.");
			eh_result = OC_EH_NOT_ACCEPTABLE;
		default:
			things_release_resource_inst(clone_resource);
			clone_resource = NULL;
			break;
		}
	} else {
		THINGS_LOG_D(TAG, "reset = %d, So, can not reset start.", reset);
		eh_result = OC_EH_OK;
	}

	return eh_result;
}

static OCEntityHandlerResult trigger_stop_ap_request(things_resource_s *target_resource, bool stop_ap)
{
	OCEntityHandlerResult eh_result = OC_EH_ERROR;

	THINGS_LOG_D(TAG, "==> STOP SOFT AP : %s", (stop_ap == true ? "YES" : "NO"));

	if (stop_ap == true) {
		if (NULL != g_stop_soft_ap_cb) {
			if (1 == g_stop_soft_ap_cb(stop_ap)) {
				THINGS_LOG_D(TAG, "Stop Soft AP notified Successfully");
				eh_result = OC_EH_OK;
			} else {
				THINGS_LOG_D(TAG, "Stop Soft AP notified, BUT DENIED");
				eh_result = OC_EH_ERROR;
			}
		}
	} else {
		THINGS_LOG_D(TAG, "stop_ap = %d, So, can not stop AP.", stop_ap);
		eh_result = OC_EH_OK;
	}

	return eh_result;
}

static OCEntityHandlerResult trigger_abort_request(things_resource_s *target_resource, things_es_enrollee_abort_e abort_es)
{
	OCEntityHandlerResult eh_result = OC_EH_ERROR;
	static pthread_t h_thread_abort = NULL;

	THINGS_LOG_D(TAG, "==> ABORT Easy Setup : %d", abort_es);

	switch (abort_es) {
	case ABORT_BEFORE_RESET_CONFIRM:	// before Reset Confirm.
	case ABORT_BEFORE_SEC_CONFIRM:	// After Reset Confirm & before Security Confirm.
	case ABORT_BEFORE_DATA_PROVISIONING:	// After Security Confirm
		THINGS_LOG_D(TAG, "Forwarding abort-level to st_things App.(Level: %d)", abort_es);
		eh_result = things_abort(&h_thread_abort, abort_es);
		break;
	default:
		THINGS_LOG_D(TAG, "abort_es = %d, So, Don't need any-process", abort_es);
		eh_result = OC_EH_OK;
		break;
	}

	return eh_result;
}

static OCEntityHandlerResult set_provisioning_info(things_resource_s *target_resource)
{
	OCEntityHandlerResult eh_result = OC_EH_ERROR;

	bool reset = false;
	bool stop_ap = false;
	int64_t abort_es = 0;

	if (target_resource->rep->things_get_bool_value(target_resource->rep, SEC_ATTRIBUTE_PROV_RESET, &reset) == true) {
		eh_result = trigger_reset_request(target_resource, reset);
	} else if (target_resource->rep->things_get_bool_value(target_resource->rep, SEC_ATTRIBUTE_PROV_TERMINATE_AP, &stop_ap) == true) {
		eh_result = trigger_stop_ap_request(target_resource, stop_ap);
#ifdef CONFIG_ST_THINGS_EASYSETUP_ABORT
	} else if (target_resource->rep->things_get_int_value(target_resource->rep, SEC_ATTRIBUTE_PROV_ABORT, &abort_es) == true) {
		eh_result = trigger_abort_request(target_resource, (things_es_enrollee_abort_e) abort_es);
#endif
	} else {
		THINGS_LOG_E(TAG, "Get Value is failed.(RESET NOR TERMINATE NOR ABORT)");
		return eh_result;
	}

	return eh_result;
}

static OCEntityHandlerResult process_post_request(things_resource_s **target_res)
{
	OCEntityHandlerResult eh_result = OC_EH_ERROR;

	things_resource_s *target_resource = *target_res;

	if (strstr(target_resource->uri, URI_SEC) != NULL && strstr(target_resource->uri, URI_PROVINFO) != NULL) {
		// 1. Post request for the Easy-Setup reset
		eh_result = set_provisioning_info(target_resource);
#ifdef CONFIG_ST_THINGS_FOTA
	} else if (strstr(target_resource->uri, URI_FIRMWARE) != NULL) {
		eh_result = fmwup_set_data(target_resource);
#endif
	} else {
		if (g_handle_request_set_cb != NULL) {
			int ret = g_handle_request_set_cb(target_resource);
			if (1 == ret) {
				eh_result = OC_EH_OK;
			} else {
				THINGS_LOG_E(TAG, "Handled as ERROR from App.");
			}
		} else {
			THINGS_LOG_E(TAG, "g_handle_request_set_cb is not registered");
		}
	}

	return eh_result;
}

static OCEntityHandlerResult process_get_request(things_resource_s *target_resource)
{

	OCEntityHandlerResult eh_result = OC_EH_ERROR;

	char *device_id = NULL;

	device_id = strrchr(target_resource->uri, '/') + 1;
	THINGS_LOG_D(TAG, "Get Device ID in Resources URI = %s", device_id);

	things_representation_s *rep = things_create_representation_inst(NULL);
	target_resource->things_set_representation(target_resource, rep);

	if (strstr(target_resource->uri, URI_SEC) != NULL && strstr(target_resource->uri, URI_PROVINFO) != NULL) {
		// 1. Get request for the Access point list
		eh_result = get_provisioning_info(target_resource);
#ifdef CONFIG_ST_THINGS_FOTA
	} else if (strstr(target_resource->uri, URI_FIRMWARE) != NULL) {
		eh_result = fmwup_get_data(target_resource);
#endif
	} else {
		if (g_handle_request_get_cb != NULL) {
			int ret = g_handle_request_get_cb(target_resource);
			if (1 == ret) {
				eh_result = OC_EH_OK;
			} else {
				THINGS_LOG_E(TAG, "Handled as ERROR from App.");
			}
		} else {
			THINGS_LOG_E(TAG, "g_handle_request_get_cb is not registered");
		}
	}

	return eh_result;
}

static bool is_can_not_response_case(things_resource_s *target_resource, OCMethod req_type, OCEntityHandlerResult eh_result)
{
	THINGS_LOG_D(TAG, THINGS_FUNC_ENTRY);

	bool res = false;

	if (target_resource == NULL) {
		THINGS_LOG_E(TAG, "Invalid argument.(target_resource=NULL)");
		return true;
	}

	THINGS_LOG_D(TAG, "target_resource->uri=%s", target_resource->uri);
	if (strstr(target_resource->uri, URI_SEC) != NULL && strstr(target_resource->uri, URI_PROVINFO) != NULL) {
		if (req_type == OC_REST_POST && eh_result == OC_EH_SLOW) {
			res = true;
		}
	}

	THINGS_LOG_D(TAG, "result = %d", res);
	THINGS_LOG_D(TAG, THINGS_FUNC_EXIT);
	return res;
}

void notify_result_of_reset(things_resource_s *target_resource, bool result)
{
	THINGS_LOG_D(TAG, THINGS_FUNC_ENTRY);

	OCEntityHandlerResult eh_result = OC_EH_ERROR;

	if (target_resource == NULL) {
		THINGS_LOG_D(TAG, "Not exist remote-client.");
		return;
	}

	if (result == true) {		// reset Success.
		OCRepPayload *rep_payload = target_resource->things_get_rep_payload(target_resource);

		eh_result = send_response(target_resource->request_handle,	// reqInfo->reqHandle,
								  target_resource->resource_handle,	// reqInfo->resHandle,
								  target_resource->error, target_resource->uri, rep_payload);

		OCPayloadDestroy((OCPayload *) rep_payload);
		rep_payload = NULL;

	} else {
		THINGS_LOG_E(TAG, "Handing Request Failed, Sending ERROR Response");

		send_response(target_resource->request_handle, target_resource->resource_handle, eh_result, target_resource->uri, NULL);
	}

	things_release_resource_inst(target_resource);

	THINGS_LOG_D(TAG, THINGS_FUNC_EXIT);
}

int notify_things_observers(const char *uri, const char *query)
{
	THINGS_LOG_D(TAG, THINGS_FUNC_ENTRY);

	int res = 0;

	THINGS_LOG_D(TAG, "uri = %s", uri);
	if (NULL != uri) {
		int remainLen = MAX_RESOURCE_LEN - 1;
		char tempUri[MAX_RESOURCE_LEN] = { 0 };

		if (NULL != g_get_notify_obs_uri) {
			char *temp = g_get_notify_obs_uri(uri, query);
			if (NULL != temp) {
				things_strncpy(tempUri, temp, remainLen);
				remainLen -= strnlen(tempUri, MAX_RESOURCE_LEN - 1);
				things_free(temp);
			}
		}

		if (strnlen(tempUri, MAX_RESOURCE_LEN - 1) < 1) {
			things_strncpy(tempUri, uri, remainLen);
			remainLen -= strnlen(tempUri, MAX_RESOURCE_LEN - 1);
		}

		THINGS_LOG_D(TAG, "%s resource notifies to observers.", tempUri);

		for (int iter = 0; iter < g_builder->res_num; iter++) {
			if (strstr(g_builder->gres_arr[iter]->uri, tempUri) != NULL) {
				OCStackResult ret2 = OCNotifyAllObservers((OCResourceHandle) g_builder->gres_arr[iter]->resource_handle,
									 OC_MEDIUM_QOS);

				THINGS_LOG_D(TAG, "%s resource has notified to observers.", g_builder->gres_arr[iter]->uri);

				if (OC_STACK_OK == ret2) {
					THINGS_LOG_V(TAG, "Success: Sent notification to Observers");
				} else {
					THINGS_LOG_V(TAG, "Failed: No Observers to notify : %d ", ret2);
				}
				res = 1;
				break;
			}
		}
	}

	THINGS_LOG_D(TAG, THINGS_FUNC_EXIT);
	return res;
}

OCEntityHandlerResult handle_message(things_resource_s *target_resource)
{
	OCEntityHandlerResult eh_result = OC_EH_ERROR;

	if (NULL == target_resource) {
		THINGS_LOG_D(TAG, "Request Item is NULL.");
		return OC_EH_ERROR;
	}

	THINGS_LOG_D(TAG, "Request Handle : %x, Resource Handle : %x", target_resource->request_handle, target_resource->resource_handle);

	if (target_resource->req_type == OC_REST_GET) {
		THINGS_LOG_V(TAG, "\t\tReq. : GET on %s", target_resource->uri);
		THINGS_LOG_V(TAG, "\t\tQuery: %s", target_resource->query);
		eh_result = process_get_request(target_resource);
	} else if (target_resource->req_type == OC_REST_POST) {
		THINGS_LOG_V(TAG, "\t\tReq. : POST on  %s", target_resource->uri);
		THINGS_LOG_V(TAG, "\t\tQuery: %s", target_resource->query);
		eh_result = process_post_request(&target_resource);
	} else {
		THINGS_LOG_E(TAG, " Invalid Request Received : %d", target_resource->req_type);
	}
	THINGS_LOG_D(TAG, " @@@@@ target_resource ->size : %d", target_resource->size);

	if (is_can_not_response_case(target_resource, target_resource->req_type, eh_result) == false) {
		if (eh_result != OC_EH_OK && eh_result != OC_EH_SLOW) {
			THINGS_LOG_E(TAG, "Handing Request Failed, Sending ERROR Response");

			send_response(target_resource->request_handle, target_resource->resource_handle, eh_result, target_resource->uri, NULL);

			eh_result = OC_EH_OK;
		} else {
			OCRepPayload *rep_payload = target_resource->things_get_rep_payload(target_resource);

			eh_result = send_response(target_resource->request_handle,	// reqInfo->reqHandle,
									  target_resource->resource_handle,	// reqInfo->resHandle,
									  target_resource->error, target_resource->uri, rep_payload);

			OCPayloadDestroy((OCPayload *) rep_payload);
			rep_payload = NULL;
		}
	}
	//  Need to design How to release memory allocated for the things_resource_s list.
	things_release_resource_inst(target_resource);

	return eh_result;
}

OCEntityHandlerResult entity_handler(OCEntityHandlerFlag flag, OCEntityHandlerRequest *entity_handler_request, void *callback)
{
	THINGS_LOG_V(TAG, THINGS_FUNC_ENTRY);
	OCEntityHandlerResult eh_result = OC_EH_ERROR;

	// Validate pointer
	if (!entity_handler_request) {
		THINGS_LOG_E(TAG, "Invalid request pointer");
		return eh_result;
	}

	const char *uri = OCGetResourceUri(entity_handler_request->resource);

	// Observe Request Handling
	if (flag & OC_OBSERVE_FLAG) {
		if (OC_OBSERVE_REGISTER == entity_handler_request->obsInfo.action) {
			THINGS_LOG_V(TAG, "Observe Requset on : %s ", uri);
			// 1. Check whether it's Observe request on the Collection Resource
			if (NULL != strstr(uri, URI_DEVICE_COL)) {
				//2. Check whether the query carriese the if=oic.if.b
				if ((strstr(entity_handler_request->query, OIC_INTERFACE_BATCH) == NULL)) {
					//3. If not batch then error
					THINGS_LOG_E(TAG, "Collection Resource Requires BATCH for Observing : %s", entity_handler_request->query);
					eh_result = OC_EH_BAD_REQ;
					goto RESPONSE_ERROR;
				} else {
					THINGS_LOG_E(TAG, "Receiving Observe Request Collection Resource");
				}
			}
		} else if (OC_OBSERVE_DEREGISTER == entity_handler_request->obsInfo.action) {
			THINGS_LOG_V(TAG, "CancelObserve Request on : %s", uri);
		}
	}
	// Get/Post Request Handling
	if (flag & OC_REQUEST_FLAG) {
		if (things_get_reset_mask(RST_CONTROL_MODULE_DISABLE) == true) {
			THINGS_LOG_V(TAG, "Control Module Disable.");
			eh_result = OC_EH_NOT_ACCEPTABLE;
		} else if ((OC_REST_GET == entity_handler_request->method)
				   || (OC_REST_POST == entity_handler_request->method)) {
			THINGS_LOG_V(TAG, "Request Handle : %x, Resource Handle : %x", entity_handler_request->requestHandle, entity_handler_request->resource);
			if (verify_request(entity_handler_request, uri, (int)entity_handler_request->method) > 0) {
				//  IoTivity Stack Destroy the payload after receving result from this function
				//       Therefore, we need to copy/clone the payload for the later use..
				things_resource_s *resource = things_create_resource_inst(entity_handler_request->requestHandle,
											  entity_handler_request->resource,
											  entity_handler_request->query,
											  entity_handler_request->payload);
				resource->things_set_dev_addr(resource, &(entity_handler_request->devAddr));
				resource->req_type = entity_handler_request->method;

				eh_result = handle_message(resource);
			} else {
				THINGS_LOG_E(TAG, "Invalid Query in the Request : %s", entity_handler_request->query);
			}
		} else if (OC_REST_DELETE == entity_handler_request->method || OC_REST_PUT == entity_handler_request->method) {
			THINGS_LOG_E(TAG, "Delete/PUT Req. Handling is not supported Yet");
		} else {
			THINGS_LOG_D(TAG, "Received unsupported method from client");
		}

	}

RESPONSE_ERROR:

	if (eh_result != OC_EH_SLOW && eh_result != OC_EH_OK) {
		//  If the result is OC_EH_ERROR, the stack will remove the
		//       received request in the stack.
		//       If the reusult is OC_EH_SLOW, then the request will be
		//       stored in the stack till the response goes out
		eh_result = send_response(entity_handler_request->requestHandle, entity_handler_request->resource, eh_result, uri, NULL);
		// Currently this code will not do anything...
		//      Need to refactor later..
	}

	THINGS_LOG_D(TAG, THINGS_FUNC_EXIT);

	return eh_result;
}

void init_handler()
{
	g_quit_flag = 0;
}

void deinit_handler()
{
	g_quit_flag = 1;

	int iter = 0;
	if (g_handle_res_list != NULL) {
		for (iter = 0; iter < g_handle_res_cnt; iter++) {
			if (g_handle_res_list[iter] != NULL) {
				things_free(g_handle_res_list[iter]);
				g_handle_res_list[iter] = NULL;
			}
		}
		things_free(g_handle_res_list);
		g_handle_res_list = NULL;
	}
	g_handle_res_cnt = 0;
}

struct things_request_handler_s *get_handler_instance()
{
	struct things_request_handler_s *handler = (things_request_handler_s *) things_malloc(sizeof(things_request_handler_s));

	if (handler == NULL) {
		THINGS_LOG_E(TAG, "Not Enough Memory");
		return NULL;
	} else {
		handler->entity_handler = &entity_handler;
		handler->init_module = &init_handler;
		handler->deinit_module = &deinit_handler;
		handler->notify_things_observers = &notify_things_observers;
		return handler;
	}
}

void release_handler_instance(struct things_request_handler_s *handler)
{
	if (handler) {
		handler->deinit_module();
		things_free(handler);
	}
}

int register_handle_request_func(handle_request_func_type get_func, handle_request_func_type set_func)
{
	if (NULL != get_func && NULL != set_func) {
		g_handle_request_get_cb = get_func;
		g_handle_request_set_cb = set_func;
	} else {
		return 0;
	}
	return 1;
}

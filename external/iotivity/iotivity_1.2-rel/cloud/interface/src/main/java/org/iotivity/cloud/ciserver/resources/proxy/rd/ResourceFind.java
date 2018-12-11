/*
 * //******************************************************************
 * //
 * // Copyright 2016 Samsung Electronics All Rights Reserved.
 * //
 * //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 * //
 * // Licensed under the Apache License, Version 2.0 (the "License");
 * // you may not use this file except in compliance with the License.
 * // You may obtain a copy of the License at
 * //
 * //      http://www.apache.org/licenses/LICENSE-2.0
 * //
 * // Unless required by applicable law or agreed to in writing, software
 * // distributed under the License is distributed on an "AS IS" BASIS,
 * // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * // See the License for the specific language governing permissions and
 * // limitations under the License.
 * //
 * //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 */
package org.iotivity.cloud.ciserver.resources.proxy.rd;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import org.iotivity.cloud.base.connector.ConnectorPool;
import org.iotivity.cloud.base.device.Device;
import org.iotivity.cloud.base.device.IRequestChannel;
import org.iotivity.cloud.base.device.IResponseEventHandler;
import org.iotivity.cloud.base.exception.ClientException;
import org.iotivity.cloud.base.exception.ServerException;
import org.iotivity.cloud.base.protocols.IRequest;
import org.iotivity.cloud.base.protocols.IResponse;
import org.iotivity.cloud.base.protocols.MessageBuilder;
import org.iotivity.cloud.base.protocols.enums.RequestMethod;
import org.iotivity.cloud.base.resource.Resource;
import org.iotivity.cloud.ciserver.Constants;
import org.iotivity.cloud.util.Cbor;

/**
 *
 * This class provides a set of APIs to send requests about find resource to
 * resource directory
 *
 */

public class ResourceFind extends Resource {
    IRequestChannel                       mASServer = null;
    IRequestChannel                       mRDServer = null;
    private Cbor<HashMap<String, Object>> mCbor     = new Cbor<>();

    public ResourceFind() {
        super(Arrays.asList(Constants.PREFIX_OIC, Constants.WELL_KNOWN_URI));

        mASServer = ConnectorPool.getConnection("account");
        mRDServer = ConnectorPool.getConnection("rd");
    }

    class AccountReceiveHandler implements IResponseEventHandler {

        private Device   mSrcDevice;
        private IRequest mRequest;

        public AccountReceiveHandler(IRequest request, Device srcDevice) {

            mSrcDevice = srcDevice;
            mRequest = request;
        }

        @Override
        public void onResponseReceived(IResponse response)
                throws ClientException {
            switch (response.getStatus()) {
                case CONTENT:
                    HashMap<String, Object> payloadData = mCbor
                            .parsePayloadFromCbor(response.getPayload(),
                                    HashMap.class);

                    ArrayList<String> devices = (ArrayList<String>) getResponseDeviceList(
                            payloadData);

                    StringBuilder additionalQuery = makeAdditionalQuery(
                            devices);
                    String uriQuery = (additionalQuery != null
                            ? additionalQuery.toString() : "")
                            + (mRequest.getUriQuery() != null
                                    ? (";" + mRequest.getUriQuery()) : "");
                    mRequest = MessageBuilder.modifyRequest(mRequest, null,
                            uriQuery, null, null);

                    mRDServer.sendRequest(mRequest, mSrcDevice);
                    break;

                default:
                    mSrcDevice.sendResponse(response);
            }
        }

        private StringBuilder makeAdditionalQuery(
                ArrayList<String> deviceList) {

            StringBuilder additionalQuery = new StringBuilder();

            if (deviceList == null) {
                return null;
            }

            if (deviceList.isEmpty()) {
                return null;
            }

            int index = deviceList.size();
            for (String device : deviceList) {
                additionalQuery.append(Constants.REQ_DEVICE_ID + "=" + device);
                if (--index > 0) {
                    additionalQuery.append(";");
                }
            }
            return additionalQuery;
        }

        @SuppressWarnings("unchecked")
        private List<String> getResponseDeviceList(
                HashMap<String, Object> payloadData) {
            ArrayList<String> deviceList = new ArrayList<>();
            if (payloadData.containsKey(Constants.REQ_DEVICE_LIST)) {
                deviceList = (ArrayList<String>) payloadData
                        .get(Constants.REQ_DEVICE_LIST);
            }
            return deviceList;
        }
    }

    @Override
    public void onDefaultRequestReceived(Device srcDevice, IRequest request)
            throws ServerException {
        if (request.getUriQuery() != null && request.getUriQueryMap()
                .containsKey(Constants.REQ_DEVICE_ID)) {

            mRDServer.sendRequest(request, srcDevice);

        } else {
            StringBuffer additionalQuery = new StringBuffer();
            additionalQuery
                    .append(Constants.USER_ID + "=" + srcDevice.getUserId());
            additionalQuery.append(";");
            additionalQuery.append(
                    Constants.REQ_MEMBER_LIST + "=" + srcDevice.getUserId());

            String uriQuery = additionalQuery.toString()
                    + (request.getUriQuery() != null
                            ? (";" + request.getUriQuery()) : "");
            StringBuffer uriPath = new StringBuffer();
            uriPath.append(Constants.PREFIX_OIC + "/");
            uriPath.append(Constants.ACL_URI + "/");
            uriPath.append(Constants.GROUP_URI + "/");
            uriPath.append(srcDevice.getUserId());

            IRequest requestToAS = MessageBuilder.createRequest(
                    RequestMethod.GET, uriPath.toString(), uriQuery);

            mASServer.sendRequest(requestToAS,
                    new AccountReceiveHandler(request, srcDevice));
        }
    }
}
/*
 * FreeRTOS HTTPS Client V1.1.2
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/**
 * @file iot_tests_https_system.c
 * @brief Full integration tests for the HTTPS Client library.
 */

/* The config header is always included first. */
#include "iot_config.h"

/* SDK initialization include. */
#include "iot_init.h"

/* HTTPS Client internal include. */
#include "private/iot_https_internal.h"

/* HTTPS Client Utils include. */
#include "iot_https_utils.h"

/* Test network header include. */
#include IOT_TEST_NETWORK_HEADER

/* Test framework includes. */
#include "unity_fixture.h"
#include "aws_test_utils.h"

/*-----------------------------------------------------------*/

/**
 * @brief The DNS resolvable host name of the server to perform testing.
 *
 * This address MUST NOT start with http:// or https://.
 */
#ifndef IOT_TEST_HTTPS_SERVER_HOST_NAME
    #define IOT_TEST_HTTPS_SERVER_HOST_NAME    "https-server.ddnsfree.com"
#endif

/**
 * @brief The socket port of the server to connect to.
 */
#ifndef IOT_TEST_HTTPS_PORT
    #if IOT_TEST_SECURED_CONNECTION == 1
        #define IOT_TEST_HTTPS_PORT    ( ( uint16_t ) 443 )
    #else
        #define IOT_TEST_HTTPS_PORT    ( ( uint16_t ) 80 )
    #endif
#endif

/**
 * @brief ALPN protocols string.
 */
#ifndef IOT_TEST_HTTPS_ALPN_PROTOCOLS
    #define IOT_TEST_HTTPS_ALPN_PROTOCOLS    NULL
#endif

/**
 * @brief Root certificate authority to verify the server connecting to.
 */
#ifndef IOT_TEST_HTTPS_ROOT_CA
    #define IOT_TEST_HTTPS_ROOT_CA                                       \
    "-----BEGIN CERTIFICATE-----\n"                                      \
    "MIIFmTCCBIGgAwIBAgIQDvt2x//ylP0SEeizF7VXBjANBgkqhkiG9w0BAQsFADBG\n" \
    "MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRUwEwYDVQQLEwxTZXJ2ZXIg\n" \
    "Q0EgMUIxDzANBgNVBAMTBkFtYXpvbjAeFw0yMDA2MDEwMDAwMDBaFw0yMTA3MDEx\n" \
    "MjAwMDBaMCQxIjAgBgNVBAMTGWh0dHBzLXNlcnZlci5kZG5zZnJlZS5jb20wggEi\n" \
    "MA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQCbSCQNQYT7BvP81lQCef4VTOGT\n" \
    "duDfh7Pu7gVT607D/vILq9hm8ghIk1vsgkekKjINPxf3S2prKOtrncdzGqNdxqkS\n" \
    "fVqfq3a5+CymRXsTxf9xe/Q4Oq1/aKn3eT/AuwAg1ylUz4qX70Z0tMwOty/A94GK\n" \
    "Lb3tD5EF6TfB/Gylf/zjEeMTZAHaUV8y0nXFLQgBhdUmJKnq2ypEqti9CbfT8P0S\n" \
    "WhAJgdIhQnJ6x3xNP/pI1exGFb/gV1r8HJri3fs1x8gAh0/EIB305U+3Txoni+H9\n" \
    "1R9IvF9vISEfcqqPXMYlzA1yVt40V1vIg2BxPLAUj9l4bZj+VIT0wlZZ7Rm/AgMB\n" \
    "AAGjggKjMIICnzAfBgNVHSMEGDAWgBRZpGYGUqB7lZI8o5QHJ5Z0W/k90DAdBgNV\n" \
    "HQ4EFgQUc2i+g265IQEz8b3Y15sviJJxnwMwQwYDVR0RBDwwOoIZaHR0cHMtc2Vy\n" \
    "dmVyLmRkbnNmcmVlLmNvbYIdd3d3Lmh0dHBzLXNlcnZlci5kZG5zZnJlZS5jb20w\n" \
    "DgYDVR0PAQH/BAQDAgWgMB0GA1UdJQQWMBQGCCsGAQUFBwMBBggrBgEFBQcDAjA7\n" \
    "BgNVHR8ENDAyMDCgLqAshipodHRwOi8vY3JsLnNjYTFiLmFtYXpvbnRydXN0LmNv\n" \
    "bS9zY2ExYi5jcmwwIAYDVR0gBBkwFzALBglghkgBhv1sAQIwCAYGZ4EMAQIBMHUG\n" \
    "CCsGAQUFBwEBBGkwZzAtBggrBgEFBQcwAYYhaHR0cDovL29jc3Auc2NhMWIuYW1h\n" \
    "em9udHJ1c3QuY29tMDYGCCsGAQUFBzAChipodHRwOi8vY3J0LnNjYTFiLmFtYXpv\n" \
    "bnRydXN0LmNvbS9zY2ExYi5jcnQwDAYDVR0TAQH/BAIwADCCAQMGCisGAQQB1nkC\n" \
    "BAIEgfQEgfEA7wB1APZclC/RdzAiFFQYCDCUVo7jTRMZM7/fDC8gC8xO8WTjAAAB\n" \
    "cnGQ0SEAAAQDAEYwRAIgKbK2KeCwB3sWftV32RDGHVe9kmRQ2B7gHaJNnBGIBCwC\n" \
    "IH2WGm4gugqTCvpz9HpBRz+XDLjm10FxW51ajrYgVWvDAHYAXNxDkv7mq0VEsV6a\n" \
    "1FbmEDf71fpH3KFzlLJe5vbHDsoAAAFycZDRIAAABAMARzBFAiBk484xaHkIANQ9\n" \
    "W2eBmtPDHPTnHp/1rcKL0VwR43yWvQIhALDiHujqaEpbXp7E2Ily1E1TaLQsetNT\n" \
    "cOsnwov+y86mMA0GCSqGSIb3DQEBCwUAA4IBAQB0NZyKsTbRpvmFJrZm5txTlWIk\n" \
    "gzI8kveFR55D4MK36fwwcFEHHyXTrvRB+OPcaUqCL2D+vXjWBnJisfW6ZdVy9QKk\n" \
    "4l6Owtd49a1/klfw1AclV0n3+HVnfefAxmO0fC3K8ExTOsFxH5rOvCSeMlEd2WEs\n" \
    "MrTLRUsDzSeRVmOGGXEbVsrvRCzWbBU5OHKUE7JOlt025Nx4cijX8lB9+8OPBGPp\n" \
    "uXZOFERKCQQ4SOJER5SxWWEohaSfC98AmlXpchTCVlxGSFHMTApq2bAO97FFOo9R\n" \
    "EQsezTSxq3ul6CZ1u8eOfCeq6gCk+tg9ootgc2DCBRRgXX919ksgWiysOfSb\n"     \
    "-----END CERTIFICATE-----"
#endif /* ifndef IOT_TEST_HTTPS_ROOT_CA */

/**
 * Client certificate and private key configurations.
 *
 * With PKCS #11 provisioning of the internally stored keys these parameters are deprecated.
 */
#ifndef IOT_TEST_HTTPS_CLIENT_CERTIFICATE
    #define IOT_TEST_HTTPS_CLIENT_CERTIFICATE    keyCLIENT_CERTIFICATE_PEM  /**< @brief Client certificate configuration. */
#endif
#ifndef IOT_TEST_HTTPS_CLIENT_PRIVATE_KEY
    #define IOT_TEST_HTTPS_CLIENT_PRIVATE_KEY    keyCLIENT_PRIVATE_KEY_PEM  /**< @brief Client private key configuration. */
#endif

/**
 * @brief Timeout in milliseconds for tests that synchronously send HTTP requests.
 *
 * This timeout encompasses the waiting time for the both sending of the request and receiving the response.
 */
#ifndef IOT_TEST_HTTPS_SYNC_TIMEOUT_MS
    #define IOT_TEST_HTTPS_SYNC_TIMEOUT_MS    ( ( uint32_t ) 10000 )
#endif

/**
 * @brief Timeout in milliseconds for tests asynchronously sending HTTP requests.
 *
 * This timeout is use to wait for the asynchronous test to finish.
 */
#ifndef IOT_TEST_HTTPS_ASYNC_TIMEOUT_MS
    #define IOT_TEST_HTTPS_ASYNC_TIMEOUT_MS    ( ( uint32_t ) 10000 )
#endif

/**
 * @brief The initial delay in milliseconds that is doubled each retry of server connection.
 */
#ifndef IOT_TEST_HTTPS_INITIAL_CONNECTION_RETRY_DELAY
    #define IOT_TEST_HTTPS_INITIAL_CONNECTION_RETRY_DELAY    ( ( uint32_t ) 300 )
#endif

/**
 * @brief The amount of times to retry the server connection if it fails.
 */
#ifndef IOT_TEST_HTTPS_CONNECTION_NUM_RETRIES
    #define IOT_TEST_HTTPS_CONNECTION_NUM_RETRIES    ( ( int32_t ) 3 )
#endif

/**
 * @brief The size of the connection user buffer to use among the tests.
 */
#define HTTPS_TEST_CONN_USER_BUFFER_SIZE    ( 512 )

/**
 * @brief The size of the request user buffer to use among the tests.
 */
#define HTTPS_TEST_REQ_USER_BUFFER_SIZE     ( 512 )

/**
 * @brief The size of the respons user buffer to use among the tests.
 */
#define HTTPS_TEST_RESP_USER_BUFFER_SIZE    ( 512 )

/**
 * @brief The size of the response body buffer to use among the tests.
 */
#define HTTPS_TEST_RESP_BODY_BUFFER_SIZE    ( 512 )

/**
 * @brief The HTTP path for the get request.
 */
#define HTTPS_TEST_GET_REQUEST_PATH         "/get"

/**
 * @brief The HTTP path for the put request.
 */
#define HTTPS_TEST_PUT_REQUEST_PATH         "/put"

/**
 * @brief The HTTP path for the post request.
 */
#define HTTPS_TEST_POST_REQUEST_PATH        "/post"

/**
 * @brief The HTTP post and put message body data.
 */
#define HTTPS_TEST_MESSAGE_BODY \
    "{\r\b"                     \
    "  \"data\":\"data\"\r\n"   \
    "}"
#define HTTPS_TEST_MESSAGE_BODY_LENGTH    ( sizeof( HTTPS_TEST_MESSAGE_BODY ) - 1 ) /**< @brief The length of the HTTP test request message body. */

/*-----------------------------------------------------------*/

/**
 * @brief These parameters get set during the asynchronous test to verify.
 */
typedef struct _asyncVerificationParams
{
    IotSemaphore_t finishedSem;                      /**< @brief This semaphore is posted to when the asynchronous response is finished. */
    uint16_t responseStatus;                         /**< @brief The HTTP response status code in the current asynchronous request. */
    uint32_t contentLength;                          /**< @brief The HTTP response body Content-Length the current asynchronous request. */
    IotHttpsReturnCode_t contentLengthReturnCode;    /**< The HTTPS Client library return code from trying to read the Content-Length value. */
    uint32_t bodyReceivedLength;                     /**< @brief The amount of HTTP response body read from the network. */
    IotHttpsReturnCode_t readResponseBodyReturnCode; /**< @brief The HTTP Client library return code from reading HTTP response body from the network. */
} _asyncVerificationParams_t;

/*-----------------------------------------------------------*/

/**
 * @brief HTTPS Client connection user buffer to share among the tests.
 */
static uint8_t _pConnUserBuffer[ HTTPS_TEST_CONN_USER_BUFFER_SIZE ] = { 0 };

/**
 * @brief HTTPS Request user buffer to share among the tests.
 */
static uint8_t _pReqUserBuffer[ HTTPS_TEST_REQ_USER_BUFFER_SIZE ] = { 0 };

/**
 * @brief HTTPS Response user buffer to share among the tests.
 */
static uint8_t _pRespUserBuffer[ HTTPS_TEST_RESP_USER_BUFFER_SIZE ] = { 0 };

/**
 * @brief HTTPS Response body buffer to share among the tests.
 */
static uint8_t _pRespBodyBuffer[ HTTPS_TEST_RESP_BODY_BUFFER_SIZE ] = { 0 };

/**
 * @brief A IotHttpsConnectionInfo_t to share among the tests.
 */
static IotHttpsConnectionInfo_t _connInfo =
{
    .pAddress             = IOT_TEST_HTTPS_SERVER_HOST_NAME,
    .addressLen           = sizeof( IOT_TEST_HTTPS_SERVER_HOST_NAME ) - 1,
    .port                 = IOT_TEST_HTTPS_PORT,
    .flags                = 0,
    .userBuffer.pBuffer   = _pConnUserBuffer,
    .userBuffer.bufferLen = sizeof( _pConnUserBuffer ),
    #if IOT_TEST_SECURED_CONNECTION == 1
        .pCaCert          = IOT_TEST_HTTPS_ROOT_CA,
        .caCertLen        = sizeof( IOT_TEST_HTTPS_ROOT_CA ),
        .pClientCert      = IOT_TEST_HTTPS_CLIENT_CERTIFICATE,
        .clientCertLen    = sizeof( IOT_TEST_HTTPS_CLIENT_CERTIFICATE ),
        .pPrivateKey      = IOT_TEST_HTTPS_CLIENT_PRIVATE_KEY,
        .privateKeyLen    = sizeof( IOT_TEST_HTTPS_CLIENT_PRIVATE_KEY ),
        .pAlpnProtocols   = IOT_TEST_HTTPS_ALPN_PROTOCOLS,
        .alpnProtocolsLen = sizeof( IOT_TEST_HTTPS_ALPN_PROTOCOLS ),
    #endif
    .pNetworkInterface    = NULL /* This will be set during TEST_SETUP(). */
};

/*-----------------------------------------------------------*/

/**
 * @brief Asynchronous #IotHttpsClientCallbacks_t.readReadyCallback implementation to share among the tests.
 */
static void _readReadyCallback( void * pPrivData,
                                IotHttpsResponseHandle_t respHandle,
                                IotHttpsReturnCode_t rc,
                                uint16_t status )
{
    /* Disable unused parameter warning. */
    ( void ) rc;

    IotHttpsReturnCode_t returnCode;
    uint32_t bodyLen = HTTPS_TEST_RESP_BODY_BUFFER_SIZE;
    _asyncVerificationParams_t * verifParams = ( _asyncVerificationParams_t * ) pPrivData;

    /* Reset the buffer we receive HTTP body into. */
    ( void ) memset( _pRespBodyBuffer, 0x00, sizeof( _pRespBodyBuffer ) );

    verifParams->responseStatus = status;
    returnCode = IotHttpsClient_ReadResponseBody( respHandle, _pRespBodyBuffer, &bodyLen );
    verifParams->readResponseBodyReturnCode = returnCode;
    verifParams->bodyReceivedLength = bodyLen;
    returnCode = IotHttpsClient_ReadContentLength( respHandle, &( verifParams->contentLength ) );
    verifParams->contentLengthReturnCode = returnCode;
}

/*-----------------------------------------------------------*/

/**
 * @brief Asynchronous #IotHttpsClientCallbacks_t.responseCompleteCallback implementation to share among the tests.
 */
static void _responseCompleteCallback( void * pPrivData,
                                       IotHttpsResponseHandle_t respHandle,
                                       IotHttpsReturnCode_t rc,
                                       uint16_t status )
{
    /* Disable unused parameter warning. */
    ( void ) respHandle;
    ( void ) rc;
    ( void ) status;

    _asyncVerificationParams_t * verifParams = ( _asyncVerificationParams_t * ) pPrivData;

    IotSemaphore_Post( &( verifParams->finishedSem ) );
}

/*-----------------------------------------------------------*/

/**
 * @brief Send a GET request synchronously and verify the results.
 */
static void _testRequestSynchronous( bool isNonPersistent,
                                     IotHttpsMethod_t method )
{
    IotHttpsReturnCode_t returnCode = IOT_HTTPS_OK;
    IotHttpsConnectionHandle_t connHandle = IOT_HTTPS_CONNECTION_HANDLE_INITIALIZER;
    IotHttpsRequestHandle_t reqHandle = IOT_HTTPS_REQUEST_HANDLE_INITIALIZER;
    IotHttpsResponseHandle_t respHandle = IOT_HTTPS_RESPONSE_HANDLE_INITIALIZER;
    IotHttpsRequestInfo_t reqInfo = IOT_HTTPS_REQUEST_INFO_INITIALIZER;
    IotHttpsResponseInfo_t respInfo = IOT_HTTPS_RESPONSE_INFO_INITIALIZER;
    IotHttpsSyncInfo_t syncRespInfo = IOT_HTTPS_SYNC_INFO_INITIALIZER;
    IotHttpsSyncInfo_t syncReqInfo = IOT_HTTPS_SYNC_INFO_INITIALIZER;
    uint32_t contentLength = 0;
    uint16_t responseStatus = 0;

    if( ( method == IOT_HTTPS_METHOD_PUT ) || ( method == IOT_HTTPS_METHOD_POST ) )
    {
        syncReqInfo.pBody = ( uint8_t * ) HTTPS_TEST_MESSAGE_BODY;
        syncReqInfo.bodyLen = HTTPS_TEST_MESSAGE_BODY_LENGTH;
    }
    else
    {
        syncReqInfo.pBody = NULL;
        syncReqInfo.bodyLen = 0;
    }

    if( method != IOT_HTTPS_METHOD_HEAD )
    {
        memset( _pRespBodyBuffer, 0, sizeof( _pRespBodyBuffer ) );
        syncRespInfo.pBody = _pRespBodyBuffer;
        syncRespInfo.bodyLen = sizeof( _pRespBodyBuffer );
    }

    if( ( method == IOT_HTTPS_METHOD_GET ) || ( method == IOT_HTTPS_METHOD_HEAD ) )
    {
        reqInfo.pPath = HTTPS_TEST_GET_REQUEST_PATH;
        reqInfo.pathLen = sizeof( HTTPS_TEST_GET_REQUEST_PATH ) - 1;
    }
    else if( method == IOT_HTTPS_METHOD_PUT )
    {
        reqInfo.pPath = HTTPS_TEST_PUT_REQUEST_PATH;
        reqInfo.pathLen = sizeof( HTTPS_TEST_PUT_REQUEST_PATH ) - 1;
    }
    else if( method == IOT_HTTPS_METHOD_POST )
    {
        reqInfo.pPath = HTTPS_TEST_POST_REQUEST_PATH;
        reqInfo.pathLen = sizeof( HTTPS_TEST_POST_REQUEST_PATH ) - 1;
    }
    else
    {
        TEST_FAIL_MESSAGE( "Unsupported method in the HTTPS Client library." );
    }

    reqInfo.pHost = IOT_TEST_HTTPS_SERVER_HOST_NAME;
    reqInfo.hostLen = sizeof( IOT_TEST_HTTPS_SERVER_HOST_NAME ) - 1;
    reqInfo.method = method;
    reqInfo.isNonPersistent = isNonPersistent;
    reqInfo.userBuffer.pBuffer = _pReqUserBuffer;
    reqInfo.userBuffer.bufferLen = sizeof( _pReqUserBuffer );
    reqInfo.isAsync = false;
    reqInfo.u.pSyncInfo = &syncReqInfo;

    respInfo.userBuffer.pBuffer = _pRespUserBuffer;
    respInfo.userBuffer.bufferLen = sizeof( _pRespUserBuffer );
    respInfo.pSyncInfo = &syncRespInfo;

    /* Initialize the request. */
    returnCode = IotHttpsClient_InitializeRequest( &reqHandle, &reqInfo );
    TEST_ASSERT_EQUAL( IOT_HTTPS_OK, returnCode );

    /* Create the HTTPS connection. */
    RETRY_EXPONENTIAL( returnCode = IotHttpsClient_Connect( &connHandle, &_connInfo ),
                       IOT_HTTPS_OK,
                       IOT_TEST_HTTPS_INITIAL_CONNECTION_RETRY_DELAY,
                       IOT_TEST_HTTPS_CONNECTION_NUM_RETRIES );
    TEST_ASSERT_EQUAL( IOT_HTTPS_OK, returnCode );

    /* Send the HTTPS request. */
    returnCode = IotHttpsClient_SendSync( connHandle, reqHandle, &respHandle, &respInfo, IOT_TEST_HTTPS_SYNC_TIMEOUT_MS );
    TEST_ASSERT_EQUAL( IOT_HTTPS_OK, returnCode );

    /* Verify the returned status code. */
    returnCode = IotHttpsClient_ReadResponseStatus( respHandle, &responseStatus );
    TEST_ASSERT_EQUAL( IOT_HTTPS_OK, returnCode );
    TEST_ASSERT_EQUAL( IOT_HTTPS_STATUS_OK, responseStatus );

    /* Verify the content length is greater than zero for a GET request. */
    returnCode = IotHttpsClient_ReadContentLength( respHandle, &contentLength );

    /* This is only verified if the read was successful. If the Content-Length header is far down in the list of
     * response headers, it may not fit into the configured header buffer space. */
    if( returnCode == IOT_HTTPS_OK )
    {
        TEST_ASSERT_GREATER_THAN( 0, contentLength );
    }

    if( method != IOT_HTTPS_METHOD_HEAD )
    {
        /* Verify the response body has some stuff in it. */
        TEST_ASSERT_GREATER_THAN( 0, strlen( ( char * ) ( respInfo.pSyncInfo->pBody ) ) );
    }

    /* Some servers do not allow requests to be persistent. Even if the server closed the connection, the connection
     * context state is controlled by the application. */
    if( isNonPersistent )
    {
        TEST_ASSERT_FALSE( connHandle->isConnected );
    }
    else
    {
        TEST_ASSERT_TRUE( connHandle->isConnected );
        IotHttpsClient_Disconnect( connHandle );
    }
}

/**
 * @brief Send a GET request asynchronously and verify the results.
 */
static void _testRequestAsynchronous( bool isNonPersistent,
                                      IotHttpsMethod_t method )
{
    IotHttpsReturnCode_t returnCode = IOT_HTTPS_OK;
    IotHttpsConnectionHandle_t connHandle = IOT_HTTPS_CONNECTION_HANDLE_INITIALIZER;
    IotHttpsRequestHandle_t reqHandle = IOT_HTTPS_REQUEST_HANDLE_INITIALIZER;
    IotHttpsResponseHandle_t respHandle = IOT_HTTPS_RESPONSE_HANDLE_INITIALIZER;
    IotHttpsRequestInfo_t reqInfo = IOT_HTTPS_REQUEST_INFO_INITIALIZER;
    IotHttpsResponseInfo_t respInfo = IOT_HTTPS_RESPONSE_INFO_INITIALIZER;
    IotHttpsAsyncInfo_t asyncRespInfo = IOT_HTTPS_ASYNC_INFO_INITIALIZER;
    _asyncVerificationParams_t verifParams = { 0 };

    asyncRespInfo.callbacks.appendHeaderCallback = NULL;
    asyncRespInfo.callbacks.writeCallback = NULL;
    asyncRespInfo.callbacks.readReadyCallback = _readReadyCallback;
    asyncRespInfo.callbacks.responseCompleteCallback = _responseCompleteCallback;
    asyncRespInfo.callbacks.connectionClosedCallback = NULL;
    asyncRespInfo.callbacks.errorCallback = NULL;
    asyncRespInfo.pPrivData = &verifParams;

    if( ( method == IOT_HTTPS_METHOD_GET ) || ( method == IOT_HTTPS_METHOD_HEAD ) )
    {
        reqInfo.pPath = HTTPS_TEST_GET_REQUEST_PATH;
        reqInfo.pathLen = sizeof( HTTPS_TEST_GET_REQUEST_PATH ) - 1;
    }
    else if( method == IOT_HTTPS_METHOD_PUT )
    {
        reqInfo.pPath = HTTPS_TEST_PUT_REQUEST_PATH;
        reqInfo.pathLen = sizeof( HTTPS_TEST_PUT_REQUEST_PATH ) - 1;
    }
    else if( method == IOT_HTTPS_METHOD_POST )
    {
        reqInfo.pPath = HTTPS_TEST_POST_REQUEST_PATH;
        reqInfo.pathLen = sizeof( HTTPS_TEST_POST_REQUEST_PATH ) - 1;
    }
    else
    {
        TEST_FAIL_MESSAGE( "Unsupported method in the HTTPS Client library." );
    }

    reqInfo.pHost = IOT_TEST_HTTPS_SERVER_HOST_NAME;
    reqInfo.hostLen = sizeof( IOT_TEST_HTTPS_SERVER_HOST_NAME ) - 1;
    reqInfo.method = method;
    reqInfo.isNonPersistent = isNonPersistent;
    reqInfo.userBuffer.pBuffer = _pReqUserBuffer;
    reqInfo.userBuffer.bufferLen = sizeof( _pReqUserBuffer );
    reqInfo.isAsync = true;
    reqInfo.u.pAsyncInfo = &asyncRespInfo;

    respInfo.userBuffer.pBuffer = _pRespUserBuffer;
    respInfo.userBuffer.bufferLen = sizeof( _pRespUserBuffer );
    respInfo.pSyncInfo = NULL;

    /* Initialize the request. */
    returnCode = IotHttpsClient_InitializeRequest( &reqHandle, &reqInfo );
    TEST_ASSERT_EQUAL( IOT_HTTPS_OK, returnCode );

    /* Create the HTTPS connection.  */
    RETRY_EXPONENTIAL( returnCode = IotHttpsClient_Connect( &connHandle, &_connInfo ),
                       IOT_HTTPS_OK,
                       IOT_TEST_HTTPS_INITIAL_CONNECTION_RETRY_DELAY,
                       IOT_TEST_HTTPS_CONNECTION_NUM_RETRIES );
    TEST_ASSERT_EQUAL( IOT_HTTPS_OK, returnCode );

    TEST_ASSERT_TRUE( IotSemaphore_Create( &( verifParams.finishedSem ), 0, 1 ) );

    if( TEST_PROTECT() )
    {
        /* Send the HTTPS request. */
        returnCode = IotHttpsClient_SendAsync( connHandle, reqHandle, &respHandle, &respInfo );
        TEST_ASSERT_EQUAL( IOT_HTTPS_OK, returnCode );

        TEST_ASSERT_TRUE( IotSemaphore_TimedWait( &( verifParams.finishedSem ), IOT_TEST_HTTPS_ASYNC_TIMEOUT_MS ) );

        /* Verify all of the parameters set during the callbacks. */
        TEST_ASSERT_EQUAL( IOT_HTTPS_STATUS_OK, verifParams.responseStatus );

        if( verifParams.contentLengthReturnCode == IOT_HTTPS_OK )
        {
            TEST_ASSERT_GREATER_THAN( 0, verifParams.contentLength );
        }

        if( method != IOT_HTTPS_METHOD_HEAD )
        {
            TEST_ASSERT_GREATER_THAN( 0, verifParams.bodyReceivedLength );
            TEST_ASSERT_EQUAL( IOT_HTTPS_OK, verifParams.readResponseBodyReturnCode );
        }

        /* Some servers do not allow requests to be persistent. Even if the server closed the connection, the connection
         * context state is controlled by the application. */
        if( isNonPersistent )
        {
            TEST_ASSERT_FALSE( connHandle->isConnected );
        }
        else
        {
            TEST_ASSERT_TRUE( connHandle->isConnected );
            IotHttpsClient_Disconnect( connHandle );
        }
    }

    IotSemaphore_Destroy( &( verifParams.finishedSem ) );
}

/*-----------------------------------------------------------*/

/**
 * @brief Test group for HTTPS Client system tests.
 */
TEST_GROUP( HTTPS_Client_System );

/*-----------------------------------------------------------*/

/**
 * @brief Test setup for the HTTP Client system tests.
 */
TEST_SETUP( HTTPS_Client_System )
{
    /* This will initialize the library before every test case, which is OK. */
    TEST_ASSERT_EQUAL_INT( true, IotSdk_Init() );
    TEST_ASSERT_EQUAL( IOT_HTTPS_OK, IotHttpsClient_Init() );

    _connInfo.pNetworkInterface = IOT_TEST_NETWORK_INTERFACE;
}

/*-----------------------------------------------------------*/

/**
 * @brief Test teardown for the HTTP Client system tests.
 */
TEST_TEAR_DOWN( HTTPS_Client_System )
{
    IotHttpsClient_Cleanup();
    IotSdk_Cleanup();
}

/*-----------------------------------------------------------*/

/**
 * @brief Test group runner for HTTPS Client System integration tests.
 */
TEST_GROUP_RUNNER( HTTPS_Client_System )
{
    RUN_TEST_CASE( HTTPS_Client_System, GetRequestSynchronousPersistent );
    RUN_TEST_CASE( HTTPS_Client_System, GetRequestSynchronousNonPersistent );
    RUN_TEST_CASE( HTTPS_Client_System, GetRequestAsynchronousPersistent );
    RUN_TEST_CASE( HTTPS_Client_System, GetRequestAsynchronousNonPersistent );
    RUN_TEST_CASE( HTTPS_Client_System, HeadRequestSynchronousPersistent );
    RUN_TEST_CASE( HTTPS_Client_System, HeadRequestSynchronousNonPersistent );
    RUN_TEST_CASE( HTTPS_Client_System, HeadRequestAsynchronousPersistent );
    RUN_TEST_CASE( HTTPS_Client_System, HeadRequestAsynchronousNonPersistent );
    RUN_TEST_CASE( HTTPS_Client_System, PutRequestSynchronousPersistent );
    RUN_TEST_CASE( HTTPS_Client_System, PutRequestSynchronousNonPersistent );
    RUN_TEST_CASE( HTTPS_Client_System, PutRequestAsynchronousPersistent );
    RUN_TEST_CASE( HTTPS_Client_System, PutRequestAsynchronousNonPersistent );
    RUN_TEST_CASE( HTTPS_Client_System, PostRequestSynchronousPersistent );
    RUN_TEST_CASE( HTTPS_Client_System, PostRequestSynchronousNonPersistent );
    RUN_TEST_CASE( HTTPS_Client_System, PostRequestAsynchronousPersistent );
    RUN_TEST_CASE( HTTPS_Client_System, PostRequestAsynchronousNonPersistent );
}

/*-----------------------------------------------------------*/

/**
 * @brief Verify a persistent GET request to the test server sent synchronously.
 */
TEST( HTTPS_Client_System, GetRequestSynchronousPersistent )
{
    _testRequestSynchronous( false, IOT_HTTPS_METHOD_GET );
}

/*-----------------------------------------------------------*/

/**
 * @brief Verify a non-persistent GET request to the test server sent synchronously.
 */
TEST( HTTPS_Client_System, GetRequestSynchronousNonPersistent )
{
    _testRequestSynchronous( true, IOT_HTTPS_METHOD_GET );
}

/*-----------------------------------------------------------*/

/**
 * @brief Verify a GET request to the test server sent asynchronously.
 */
TEST( HTTPS_Client_System, GetRequestAsynchronousPersistent )
{
    _testRequestAsynchronous( false, IOT_HTTPS_METHOD_GET );
}

/*-----------------------------------------------------------*/

/**
 * @brief Verify a GET request to the test server sent asynchronously.
 */
TEST( HTTPS_Client_System, GetRequestAsynchronousNonPersistent )
{
    _testRequestAsynchronous( true, IOT_HTTPS_METHOD_GET );
}

/*-----------------------------------------------------------*/

/**
 * @brief Verify a persistent HEAD request to the test server sent synchronously.
 */
TEST( HTTPS_Client_System, HeadRequestSynchronousPersistent )
{
    _testRequestSynchronous( false, IOT_HTTPS_METHOD_HEAD );
}

/*-----------------------------------------------------------*/

/**
 * @brief Verify a non-persistent HEAD request to the test server sent synchronously.
 */
TEST( HTTPS_Client_System, HeadRequestSynchronousNonPersistent )
{
    _testRequestSynchronous( true, IOT_HTTPS_METHOD_HEAD );
}

/*-----------------------------------------------------------*/

/**
 * @brief Verify a HEAD request to the test server sent asynchronously.
 */
TEST( HTTPS_Client_System, HeadRequestAsynchronousPersistent )
{
    _testRequestAsynchronous( false, IOT_HTTPS_METHOD_HEAD );
}

/*-----------------------------------------------------------*/

/**
 * @brief Verify a HEAD request to the test server sent asynchronously.
 */
TEST( HTTPS_Client_System, HeadRequestAsynchronousNonPersistent )
{
    _testRequestAsynchronous( true, IOT_HTTPS_METHOD_HEAD );
}

/*-----------------------------------------------------------*/

/**
 * @brief Verify a persistent PUT request to the test server sent synchronously.
 */
TEST( HTTPS_Client_System, PutRequestSynchronousPersistent )
{
    _testRequestSynchronous( false, IOT_HTTPS_METHOD_PUT );
}

/*-----------------------------------------------------------*/

/**
 * @brief Verify a non-persistent PUT request to the test server sent synchronously.
 */
TEST( HTTPS_Client_System, PutRequestSynchronousNonPersistent )
{
    _testRequestSynchronous( true, IOT_HTTPS_METHOD_PUT );
}

/*-----------------------------------------------------------*/

/**
 * @brief Verify a PUT request to the test server sent asynchronously.
 */
TEST( HTTPS_Client_System, PutRequestAsynchronousPersistent )
{
    _testRequestAsynchronous( false, IOT_HTTPS_METHOD_PUT );
}

/*-----------------------------------------------------------*/

/**
 * @brief Verify a PUT request to the test server sent asynchronously.
 */
TEST( HTTPS_Client_System, PutRequestAsynchronousNonPersistent )
{
    _testRequestAsynchronous( true, IOT_HTTPS_METHOD_PUT );
}

/*-----------------------------------------------------------*/

/**
 * @brief Verify a persistent POST request to the test server sent synchronously.
 */
TEST( HTTPS_Client_System, PostRequestSynchronousPersistent )
{
    _testRequestSynchronous( false, IOT_HTTPS_METHOD_POST );
}

/*-----------------------------------------------------------*/

/**
 * @brief Verify a non-persistent POST request to the test server sent synchronously.
 */
TEST( HTTPS_Client_System, PostRequestSynchronousNonPersistent )
{
    _testRequestSynchronous( true, IOT_HTTPS_METHOD_POST );
}

/*-----------------------------------------------------------*/

/**
 * @brief Verify a POST request to the test server sent asynchronously.
 */
TEST( HTTPS_Client_System, PostRequestAsynchronousPersistent )
{
    _testRequestAsynchronous( false, IOT_HTTPS_METHOD_POST );
}

/*-----------------------------------------------------------*/

/**
 * @brief Verify a POST request to the test server sent asynchronously.
 */
TEST( HTTPS_Client_System, PostRequestAsynchronousNonPersistent )
{
    _testRequestAsynchronous( true, IOT_HTTPS_METHOD_POST );
}

// Minimal stubs to satisfy ROS-packaged static Azure IoT SDK link dependencies.
// This receiver uses IoTHubDeviceClient_CreateFromConnectionString and does not
// rely on the provisioning/HSM backends provided by libhsm_security_client.
extern "C" {

int initialize_hsm_system(void)
{
  return 0;
}

void deinitialize_hsm_system(void)
{
}

const void * hsm_client_x509_interface(void)
{
  return nullptr;
}

const void * hsm_client_key_interface(void)
{
  return nullptr;
}

const void * hsm_client_tpm_interface(void)
{
  return nullptr;
}

}  // extern "C"

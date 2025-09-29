//String ThingId="funnelbm0525A006";
//String ThingId="ELEVATE_SW_DEVICE_MAIN";
String ThingId = "funnelbm0925A001";
String DeviceId=  "IOTIQBM_A0525001";
//String DeviceId="BASE_MOD";
String BLE_name="IOTIQBM";
String Secret="xxxxxxxxxxxxx";
//String Mqtt_server="a1r6z29mxc63px-ats.iot.ap-south-1.amazonaws.com";
// String Mqtt_server="a1r6z29mxc63px-ats.iot.ap-south-1.amazonaws.com";
String Mqtt_server ="a1r6z29mxc63px-ats.iot.eu-west-1.amazonaws.com";
String OTA_host="";
String FirmwareVer = "1.0.0"; 
String Access_token = "";
String ssid = "R&D1";
String password = "RND@1980";
String Version_device="";
bool versionflag = false;
String Lora_data="";
bool Lora_flag=0;
bool motor1_flag;
bool motor2_flag;
// Publish Topics
String UpdateTopic = "";
String OtavalidateTopic = "";
String AliveTopic = "";
String HealthTopic = "";
String SlaveTopic="";
String StatusTopic = "";

// Subscribe Topics
String SettingTopic = "";
String ConfigTopic = "";
String ControlTopic = "";
String isAliveTopic = "";
String isHealthTopic = "";
String OtaintializeTopic = "";
String OtarequestTopic="";
String ResetTopic = "";
String SlaverequestTopic="";
String StatusrequestTopic = "";
String DeleteSlaveTopic = "";

const byte pinLength = 3;
const int pins[pinLength] = { 14, 21, 22};
const int input[pinLength] = { 15, 4};
const int touch[pinLength] = { 32, 33 };
const int echo =27;
const int trig=26;
#define RX2 16
#define TX2 17
#define NUM_LEDS 13
#define DATA_PIN 13

const int M0=18;
const int M1=19;
const int AUX=23;
const int RXG=2;
const int TXG=5;
const int channel=23;
const int address_l=0x01;
const int address_h=0x00;

byte  inputs[pinLength]       = {0, 0};
bool  power_status[pinLength] = {0, 0};
byte  inputs_last[pinLength]  = {0, 0};
String relays[pinLength]      = {"S1", "S2","S3"};
String relays_setup[2]        = {" ", " "}; // two tanks map to relay labels
String slave_name[20] = {"TM1","TM2","TM3","TM4","MC1","MC2","MC3","MC4","VC1","VC2","VC3","VC4","WC1","WC2"};
int number=0;
String tank_name[4] = {"TM1","TM2","TM3","TM4"};
String motor_name[4] = {"MC1","MC2","MC3","MC4"};
//const char* tankNames[4] = {"TM1","TM2","TM3","TM4"};
int tanks_set[2] = {0, 0};
int tanks_trig[2] = {0, 0};
int tanks_stop[2] = {0, 0};

// Root CA, Client Certificate, and Client Private Key
String rootCACertificate = R"EOF(-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----)EOF";
String clientCertificate = R"KEY(-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUQ8MtucO3TfF2ro6DfsYwhYQ7rF0wDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI1MDkyNDA2MTQ1
NloXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALqOV9cucYW8MV6vHrbt
zNDvGak+wDVX6rghQHFbjRN7YTId13e7yzImTurFUqNciZ+e5uW673J2Hs7Q4NEP
FDxM3Y5pS34aaVKY0dSDrzAtX761hpmULEu/qn58LZ4E69ymSK+W6WB9Oz5y7hRy
ERfVJlAuc0yN7+9AwGDFMrhZeoKLuIsDvrm1W0veSM0hRw3Sajgh0B57XQqa88Eu
/05luLSP+7FRBGeeNKbyqJMwcPfskHpKZJrF/na7b8z4px8UHIo4lwZL2DMejBuh
BX14QSNWH6u9STO4EQYxBGpGoaeMmzZIN8V59V7x9eIo2KuIZNoNP4I9riyY4MMO
DR8CAwEAAaNgMF4wHwYDVR0jBBgwFoAU6NF3G16yJgD6cBXPHDlgD0O3rC4wHQYD
VR0OBBYEFA/izithNxOAVJNS5re0yCZRk1JuMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQCZwStwYRrjyPAVArE9Fc0wWT9V
3LY2S2nFBULBAMeIYwadze4ZsGmxmsjjVAaykEV3IlyeausB5JarFI3itux+Z7KB
WVTG5dJlCZDgtI4juLTHj6Eokq//yv8VkOsb2E7OzxhPSfA403AMqiCSjD/xZH65
nn3f3h27A/8B8KSu0C2YbBHoD0jwzP6+NIlMpbNAs0FiOqHeRrA4YXVKqGpAdrm0
mAR5ViKY/t7ZFWRGergPZK4Y01DF5THLa8840Rb8cwivZTQL+yI8rP1hTKqGGIHe
ChBj5fJZ/kSzxsxYJGupIRYBk8XS3bAqjE+Q+NUkHOp3kuXolCCWOZOY2N72
-----END CERTIFICATE-----
)KEY";
String clientPrivateKey = R"KEY(-----BEGIN RSA PRIVATE KEY-----
MIIEpAIBAAKCAQEAuo5X1y5xhbwxXq8etu3M0O8ZqT7ANVfquCFAcVuNE3thMh3X
d7vLMiZO6sVSo1yJn57m5brvcnYeztDg0Q8UPEzdjmlLfhppUpjR1IOvMC1fvrWG
mZQsS7+qfnwtngTr3KZIr5bpYH07PnLuFHIRF9UmUC5zTI3v70DAYMUyuFl6gou4
iwO+ubVbS95IzSFHDdJqOCHQHntdCprzwS7/TmW4tI/7sVEEZ540pvKokzBw9+yQ
ekpkmsX+drtvzPinHxQcijiXBkvYMx6MG6EFfXhBI1Yfq71JM7gRBjEEakahp4yb
Nkg3xXn1XvH14ijYq4hk2g0/gj2uLJjgww4NHwIDAQABAoIBAQCPgdwUjYeKZuBb
g77O3VMHDkbhKIJXBpBqoDmgedqmN+zmzonnczC2HT3r32rYe+DqRyQ7aHe1xyOf
2JJ4f28Di+WLHXiTQuHuRdxz6/Ch0OWsIJuwHImOo5wVvJSi0ApYLBoR6LcSIIOo
YGTAmZaIjNDyMSlEpJMDqv3R8yA3Kl2O1UF69OntdzrRuzNJLLJgIIgu1pTfyXGQ
j1rsUWIcSxnxfQzSfUaIVA/+OMX4U+HG6TGpji1/ZB+laQv03KY1V8WydhCP6Vpu
yj4Gn+Mk5SIKykhL69JmC1EkY7ypp4Am2aJAlFc8XBPHMGhFs6IA5euwzLdtKxHm
5FzxhecBAoGBAN3fGizOn8KhzDXPZhmTBZGWVlHiFYFhztM/UtXAZZTCvLXWbQUi
UiUHOXOPU/R8yRS3xxeCg0VmdygKJDQnGRGgmqWQVy9QezB37V5KoIA0jkUa0lJh
TAed/4KkO+ubst8orTA3gP3jTNCAjcIoQS5xVPADQ+nJ1QVn50j7Xp/BAoGBANdA
lPAGQKM/YYCub7rHBYU+BzWJnaupZX2lyAss+Tz6Ra9p4xIajr8C/JVeoEwGREqd
qs56EZwx/ozEB5S0oHg+vHAMb8FbRosSDVrdFoeYrVa8b28NlmzZhfF5r/M7sQJ/
ERhVz7UNAd52JnT1AK4yE5/yesSAaBhhyLLLmeTfAoGACS5qTuBeDBfV84QZnYVP
/a5S9CB+81Ow010TeHT1vyov1PaCnGvHKnEaF4Ye20cuqwTP4FEuTXjoBWgmB2J2
zxvuSlumv/Z8oozT7cr4yVVjbcimW3JbSxVmtWlGcYGH7WQpho1FcTwuuZIYS6iO
yfU2ppf7/3iyr/6Uu5hrWgECgYAFfq+OpQo3YKHTkCBoCzaX3Sp/8mBlBnEB0R6X
MnG2XckznideyfDE7YWXJpA/AJXztayrkrAqZZhS7Zon8Kh8CVX0Ik0kCXl2iWJv
5F7z3TDnmu3ZTuZ9Jtxleq7ELczp/GZqUZ54x7k5fZijubFnwP2BQsZhh8mgfbaK
4emuzwKBgQCUcRe85bGwLfX9SaL+SR1lWnb14BrJM0ELiTgeOfWA9elm09qGyX4w
HiHGLTyTPWzfv1KBKu1GQLRKL3XVGXJUixqeVO7thFuY4jF0eW6jY52dL7SY3itW
EnSaMb3+o2TLYdB9Ml2anz6lIUEA4wPgCPLbnlcMqKkcvmrhJOGxFw==
-----END RSA PRIVATE KEY-----
)KEY";
String clientPublicKey = R"KEY(-----BEGIN PUBLIC KEY-----
MIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuo5X1y5xhbwxXq8etu3M
0O8ZqT7ANVfquCFAcVuNE3thMh3Xd7vLMiZO6sVSo1yJn57m5brvcnYeztDg0Q8U
PEzdjmlLfhppUpjR1IOvMC1fvrWGmZQsS7+qfnwtngTr3KZIr5bpYH07PnLuFHIR
F9UmUC5zTI3v70DAYMUyuFl6gou4iwO+ubVbS95IzSFHDdJqOCHQHntdCprzwS7/
TmW4tI/7sVEEZ540pvKokzBw9+yQekpkmsX+drtvzPinHxQcijiXBkvYMx6MG6EF
fXhBI1Yfq71JM7gRBjEEakahp4ybNkg3xXn1XvH14ijYq4hk2g0/gj2uLJjgww4N
HwIDAQAB
-----END PUBLIC KEY-----
)KEY";




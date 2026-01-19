# Webprov Web Provisioning configuration parameters

In our previous session we added web provisioning to this project.
We need to add 2 more configuraiton variables.

Please read specs/webprov.md. Note the payload of the /config POST endpoint now has 2 more parameters: device_name and actuator.

Add them to both the html page as well as to the /config POST service endpoint.
Note that these new variables are NOT persisted in nvs namespace 'config' but rather in namespace 'meta'!

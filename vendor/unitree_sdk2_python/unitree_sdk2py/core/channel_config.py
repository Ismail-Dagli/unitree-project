ChannelConfigHasInterface = '''<?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
        <Domain Id="any">
            <General>
                <Interfaces>
                    <NetworkInterface name="$__IF_NAME__$" priority="default" multicast="true"/>
                </Interfaces>
                <AllowMulticast>true</AllowMulticast>
            </General>
            <Discovery>
                <ParticipantIndex>auto</ParticipantIndex>
                <Peers>
                    <Peer address="192.168.123.161"/>
                    <Peer address="192.168.123.164"/>
                </Peers>
            </Discovery>
            <Tracing>
                <Verbosity>finest</Verbosity>
                <OutputFile>/tmp/cdds.LOG</OutputFile>
            </Tracing>
        </Domain>
    </CycloneDDS>'''

ChannelConfigAutoDetermine = '''<?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
        <Domain Id="any">
            <General>
                <Interfaces>
                    <NetworkInterface autodetermine=\"true\" priority=\"default\" multicast=\"default\" />
                </Interfaces>
            </General>
        </Domain>
    </CycloneDDS>'''

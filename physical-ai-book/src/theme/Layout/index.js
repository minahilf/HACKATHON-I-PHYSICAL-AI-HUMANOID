import React from 'react';
import Layout from '@theme-original/Layout';
import AIChat from '@site/src/components/AIChat';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props} />
      <AIChat />
    </>
  );
}
